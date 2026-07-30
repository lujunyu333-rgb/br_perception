#include "br_perception/lidar/cluster_extractor.hpp"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <chrono>
#include <cmath>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>      // std::pair

namespace br_perception {
namespace lidar {

// ═══════════════════════════════════════════════════════════════════════════
// 辅助: ObstacleType → 字符串
// ═══════════════════════════════════════════════════════════════════════════
namespace {
const char* obstacle_type_name(ObstacleType t) {
  switch (t) {
    case ObstacleType::EARTH_CUBE:   return "Earth Cube";
    case ObstacleType::SKY_CUBE:     return "Sky Cube";
    case ObstacleType::ENEMY_ROBOT:  return "Enemy Robot";
    case ObstacleType::MUSTIKA:      return "Mustika";
    default:                         return "Unknown";
  }
}
}  // anonymous namespace

// ═══════════════════════════════════════════════════════════════════════════
// 构造 & 参数初始化
// ═══════════════════════════════════════════════════════════════════════════
ClusterExtractor::ClusterExtractor(const rclcpp::NodeOptions& options)
  : Node("cluster_extractor", options)
{
  // ── 声明聚类参数 ──
  this->declare_parameter<double>("cluster_tolerance",    cluster_tolerance_);
  this->declare_parameter<int>("min_cluster_size",        min_cluster_size_);
  this->declare_parameter<int>("max_cluster_size",        max_cluster_size_);

  // ── 声明过滤参数 ──
  this->declare_parameter<double>("max_object_size",      max_object_size_);
  this->declare_parameter<double>("max_object_height",    max_object_height_);
  this->declare_parameter<int>("min_object_points",       min_object_points_);
  this->declare_parameter<double>("floating_z_min",       floating_z_min_);

  // ── 声明圆柱掩膜参数 ──
  this->declare_parameter<bool>("enable_cylinder_mask",   enable_cylinder_mask_);
  this->declare_parameter<double>("cylinder_mask_radius", cylinder_mask_radius_);
  this->declare_parameter<std::vector<double>>("cylinder_mask_x");
  this->declare_parameter<std::vector<double>>("cylinder_mask_y");

  // ── 声明分类参数 ──
  this->declare_parameter<double>("size_tolerance",       size_tolerance_);
  this->declare_parameter<double>("earth_cube_size",      earth_cube_size_);
  this->declare_parameter<double>("sky_cube_size",        sky_cube_size_);
  this->declare_parameter<double>("enemy_robot_size",     enemy_robot_size_);
  this->declare_parameter<double>("mustika_size",         mustika_size_);
  this->declare_parameter<double>("elongation_ratio",     elongation_ratio_);
  this->declare_parameter<double>("sphere_eigen_tol",     sphere_eigen_tol_);

  // ── 读取参数 ──
  #define GET(name, var) this->get_parameter(name, var)
  GET("cluster_tolerance",    cluster_tolerance_);
  GET("min_cluster_size",     min_cluster_size_);
  GET("max_cluster_size",     max_cluster_size_);
  GET("max_object_size",      max_object_size_);
  GET("max_object_height",    max_object_height_);
  GET("min_object_points",    min_object_points_);
  GET("floating_z_min",       floating_z_min_);
  GET("enable_cylinder_mask", enable_cylinder_mask_);
  GET("cylinder_mask_radius", cylinder_mask_radius_);
  GET("size_tolerance",       size_tolerance_);
  GET("earth_cube_size",      earth_cube_size_);
  GET("sky_cube_size",        sky_cube_size_);
  GET("enemy_robot_size",     enemy_robot_size_);
  GET("mustika_size",         mustika_size_);
  GET("elongation_ratio",     elongation_ratio_);
  GET("sphere_eigen_tol",     sphere_eigen_tol_);
  #undef GET

  // 圆柱中心列表: 从两个独立数组重建为 pair 向量 (避免越界)
  {
    std::vector<double> cx, cy;
    this->get_parameter("cylinder_mask_x", cx);
    this->get_parameter("cylinder_mask_y", cy);
    if (cx.size() != cy.size()) {
      RCLCPP_WARN(this->get_logger(),
        "cylinder_mask_x (%zu) and cylinder_mask_y (%zu) length mismatch; "
        "cylinder mask disabled",
        cx.size(), cy.size());
      enable_cylinder_mask_ = false;
    } else {
      cylinder_centers_.reserve(cx.size());
      for (size_t i = 0; i < cx.size(); ++i) {
        cylinder_centers_.emplace_back(cx[i], cy[i]);
      }
    }
  }

  // ── 动态参数回调 ──
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ClusterExtractor::on_param_change, this, std::placeholders::_1));

  // ── 订阅 ──
  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/perception/lidar/non_ground",
    rclcpp::SensorDataQoS(),
    std::bind(&ClusterExtractor::cloud_callback, this, std::placeholders::_1));

  // ── 发布 ──
  pub_obstacles_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/perception/lidar/obstacles", 10);
  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/perception/lidar/diagnostics", 10);

  RCLCPP_INFO(this->get_logger(),
    "ClusterExtractor ready | "
    "cluster: tol=%.3fm min=%d max=%d | "
    "filter: max_size=%.2fm max_h=%.2fm min_pts=%d floating_z=%.2fm | "
    "cylinder_mask: %s radius=%.2fm centers=%zu | "
    "classify: tol=%.3fm earth=%.2f sky=%.2f robot=%.2f mustika=%.2f",
    cluster_tolerance_, min_cluster_size_, max_cluster_size_,
    max_object_size_, max_object_height_, min_object_points_, floating_z_min_,
    enable_cylinder_mask_ ? "ON" : "OFF",
    cylinder_mask_radius_, cylinder_centers_.size(),
    size_tolerance_, earth_cube_size_, sky_cube_size_,
    enemy_robot_size_, mustika_size_);
}

// ═══════════════════════════════════════════════════════════════════════════
// 主回调
// ═══════════════════════════════════════════════════════════════════════════
void ClusterExtractor::cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto t0 = std::chrono::steady_clock::now();

  // ── 解包 ──
  PointCloudPtr cloud_in = std::make_shared<PointCloud>();
  pcl::fromROSMsg(*msg, *cloud_in);

  if (cloud_in->size() < static_cast<size_t>(min_cluster_size_)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                         3000, "Too few non-ground points: %zu", cloud_in->size());
    // 即使无有效数据也发布空结果, 避免下游等待超时
    std::vector<Obstacle> empty;
    publish_obstacles(empty, msg->header);
    publish_diagnostics(0.0, 0, 0, empty);
    return;
  }

  // ── 1. 圆柱掩膜 (可选) ──
  PointCloudPtr cloud_work = cloud_in;
  if (enable_cylinder_mask_ && !cylinder_centers_.empty()) {
    cloud_work = mask_cylinders(cloud_in);
  }

  // ── 2. 欧式聚类 ──
  auto clusters = extract_clusters(cloud_work);
  const size_t cluster_count = clusters.size();

  // ── 3. 对每个聚类计算特征 ──
  std::vector<Obstacle> raw_obstacles;
  raw_obstacles.reserve(cluster_count);
  for (size_t i = 0; i < cluster_count; ++i) {
    auto obs = compute_obstacle(cloud_work, clusters[i], static_cast<int>(i));
    raw_obstacles.push_back(obs);
  }

  // ── 4. 规则过滤 + 5. 分类猜测 ──
  std::vector<Obstacle> passed;
  passed.reserve(raw_obstacles.size());

  for (auto& obs : raw_obstacles) {
    if (filter_obstacle(obs)) continue;
    classify_obstacle(obs);
    passed.push_back(obs);
  }

  // ── 6. 重新编排 ID (按通过顺序) ──
  for (size_t i = 0; i < passed.size(); ++i) {
    passed[i].id = static_cast<int>(i);
  }

  // ── 耗时 ──
  auto t_end = std::chrono::steady_clock::now();
  using ms = std::chrono::duration<float, std::milli>;
  double total_ms = ms(t_end - t0).count();

  // ── 日志摘要 ──
  RCLCPP_DEBUG(this->get_logger(),
    "Clusters: raw=%zu passed=%zu | %.1f ms",
    cluster_count, passed.size(), total_ms);

  // ── 7. 发布 ──
  publish_obstacles(passed, msg->header);
  publish_diagnostics(total_ms, cluster_count, passed.size(), passed);
}

// ═══════════════════════════════════════════════════════════════════════════
// 欧式聚类
// ═══════════════════════════════════════════════════════════════════════════
std::vector<pcl::PointIndices>
ClusterExtractor::extract_clusters(const PointCloudPtr& cloud)
{
  std::vector<pcl::PointIndices> clusters;

  if (cloud->size() < static_cast<size_t>(min_cluster_size_)) {
    return clusters;
  }

  // KdTree
  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);

  try {
    ec.extract(clusters);
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
      "EuclideanClusterExtraction exception: %s", e.what());
  }
  catch (...) {
    RCLCPP_ERROR(this->get_logger(),
      "EuclideanClusterExtraction unknown exception");
  }

  return clusters;
}

// ═══════════════════════════════════════════════════════════════════════════
// 单聚类 → Obstacle
// ═══════════════════════════════════════════════════════════════════════════
Obstacle ClusterExtractor::compute_obstacle(
    const PointCloudPtr& cloud,
    const pcl::PointIndices& cluster,
    int id)
{
  Obstacle obs;
  obs.id = id;

  const auto& indices = cluster.indices;
  if (indices.empty()) return obs;

  // ── 3D 包围盒 (min_pt, max_pt) ──
  float xmin = 1e9, ymin = 1e9, zmin = 1e9;
  float xmax = -1e9, ymax = -1e9, zmax = -1e9;
  float sx = 0, sy = 0, sz = 0;

  for (int idx : indices) {
    const auto& pt = cloud->points[idx];
    if (pt.x < xmin) xmin = pt.x;
    if (pt.y < ymin) ymin = pt.y;
    if (pt.z < zmin) zmin = pt.z;
    if (pt.x > xmax) xmax = pt.x;
    if (pt.y > ymax) ymax = pt.y;
    if (pt.z > zmax) zmax = pt.z;
    sx += pt.x;
    sy += pt.y;
    sz += pt.z;
  }

  obs.min_x = xmin; obs.max_x = xmax;
  obs.min_y = ymin; obs.max_y = ymax;
  obs.min_z = zmin; obs.max_z = zmax;

  obs.length = xmax - xmin;
  obs.width  = ymax - ymin;
  obs.height = zmax - zmin;

  // ── 质心 ──
  const float n = static_cast<float>(indices.size());
  obs.centroid_x = sx / n;
  obs.centroid_y = sy / n;
  obs.centroid_z = sz / n;

  // ── 点数 ──
  obs.point_count = static_cast<int>(indices.size());

  // ── PCA 方向 ──
  compute_pca_orientation(cloud, cluster, obs);

  return obs;
}

// ═══════════════════════════════════════════════════════════════════════════
// 3D PCA: 主轴方向 + 长宽比判断
//
// 协方差矩阵 (3×3):
//   C_ij = Σ (p_i - μ_i)(p_j - μ_j) / N
//
// 特征值 λ1≥λ2≥λ3:
//   λ1/λ2 > elongation_ratio → 长条形, 主轴 = v1
//   λ1≈λ2≈λ3             → 近似球体
//
// 偏航角: yaw = atan2(v1.y, v1.x)  (水平投影)
// ═══════════════════════════════════════════════════════════════════════════
void ClusterExtractor::compute_pca_orientation(
    const PointCloudPtr& cloud,
    const pcl::PointIndices& cluster,
    Obstacle& obs)
{
  const auto& indices = cluster.indices;
  const size_t n = indices.size();

  if (n < 3) {
    obs.yaw = 0;
    obs.is_elongated = false;
    obs.eigen_ratio_1 = 1.0f;
    obs.eigen_ratio_2 = 1.0f;
    return;
  }

  // ── 协方差矩阵 (3×3) ──
  double cov[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  const double cx = static_cast<double>(obs.centroid_x);
  const double cy = static_cast<double>(obs.centroid_y);
  const double cz = static_cast<double>(obs.centroid_z);
  const double inv_n = 1.0 / static_cast<double>(n);

  for (int idx : indices) {
    const auto& pt = cloud->points[idx];
    double dx = pt.x - cx;
    double dy = pt.y - cy;
    double dz = pt.z - cz;
    cov[0][0] += dx * dx;  cov[0][1] += dx * dy;  cov[0][2] += dx * dz;
    cov[1][0] += dy * dx;  cov[1][1] += dy * dy;  cov[1][2] += dy * dz;
    cov[2][0] += dz * dx;  cov[2][1] += dz * dy;  cov[2][2] += dz * dz;
  }

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      cov[i][j] *= inv_n;

  // ── Eigen 分解 ──
  Eigen::Matrix3d C;
  C << cov[0][0], cov[0][1], cov[0][2],
       cov[1][0], cov[1][1], cov[1][2],
       cov[2][0], cov[2][1], cov[2][2];

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(C);
  if (solver.info() != Eigen::Success) {
    obs.yaw = 0;
    obs.is_elongated = false;
    obs.eigen_ratio_1 = 1.0f;
    obs.eigen_ratio_2 = 1.0f;
    return;
  }

  // 特征值: 升序排列 (SelfAdjointEigenSolver 默认升序)
  // eigenvalues()[0] = λ3 (最小), eigenvalues()[2] = λ1 (最大)
  double lambda1 = solver.eigenvalues()[2];  // 最大
  double lambda2 = solver.eigenvalues()[1];
  double lambda3 = solver.eigenvalues()[0];  // 最小

  // 特征向量: 第 2 列 = 最大特征值对应的向量 (主轴)
  Eigen::Vector3d v1 = solver.eigenvectors().col(2);

  // ── 记录 ──
  obs.eigen_ratio_1 = (lambda2 > 1e-9)
    ? static_cast<float>(lambda1 / lambda2) : 1.0f;
  obs.eigen_ratio_2 = (lambda3 > 1e-9)
    ? static_cast<float>(lambda2 / lambda3) : 1.0f;

  // ── 判断是否长条形 ──
  obs.is_elongated = (obs.eigen_ratio_1 > elongation_ratio_);

  // ── 偏航角 (主轴水平分量) ──
  obs.yaw = std::atan2(static_cast<float>(v1.y()),
                       static_cast<float>(v1.x()));
}

// ═══════════════════════════════════════════════════════════════════════════
// 规则过滤 (ROBOCON 2027 场景)
//
// 过滤条件:
//   - 任一维度 > max_object_size_ → 围栏/墙面 (过大)
//   - 高度 > max_object_height_   → 柱子 (过高)
//   - 点数 < min_object_points_   → 噪点 (过小)
//   - 底部 z > floating_z_min_    → 悬空物 (底部无支撑)
//
// @return true = 应剔除
// ═══════════════════════════════════════════════════════════════════════════
bool ClusterExtractor::filter_obstacle(const Obstacle& obs) const
{
  // 过大物体 (围栏 / 墙面)
  if (obs.length > max_object_size_ || obs.width > max_object_size_) {
    RCLCPP_DEBUG(this->get_logger(),
      "Filtered LARGE: %.2f x %.2f x %.2f at (%.2f, %.2f)",
      obs.length, obs.width, obs.height, obs.centroid_x, obs.centroid_y);
    return true;
  }

  // 过高物体 (柱子)
  if (obs.height > max_object_height_) {
    RCLCPP_DEBUG(this->get_logger(),
      "Filtered TALL: h=%.2f at (%.2f, %.2f)",
      obs.height, obs.centroid_x, obs.centroid_y);
    return true;
  }

  // 过小物体 (噪点)
  if (obs.point_count < min_object_points_) {
    return true;
  }

  // 悬空物 (底部无支撑)
  if (obs.min_z > floating_z_min_) {
    RCLCPP_DEBUG(this->get_logger(),
      "Filtered FLOATING: z_min=%.2f at (%.2f, %.2f)",
      obs.min_z, obs.centroid_x, obs.centroid_y);
    return true;
  }

  return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// 分类猜测
//
// 基于包围盒尺寸和 PCA 特征值:
//   - 三轴 ≈ earth_cube_size_   → EARTH_CUBE   (350mm 立方体)
//   - 三轴 ≈ sky_cube_size_     → SKY_CUBE     (200mm 立方体)
//   - 三轴 ≈ enemy_robot_size_  → ENEMY_ROBOT  (700mm 立方体)
//   - 三轴 ≈ mustika_size_ 且特征值接近 → MUSTIKA (200mm 球体)
// ═══════════════════════════════════════════════════════════════════════════
void ClusterExtractor::classify_obstacle(Obstacle& obs)
{
  const float tol = static_cast<float>(size_tolerance_);

  // 三个维度的平均值和最大偏差
  const float dims[3] = { obs.length, obs.width, obs.height };
  const float avg_dim = (dims[0] + dims[1] + dims[2]) / 3.0f;
  const float max_dev = std::max({
    std::abs(dims[0] - avg_dim),
    std::abs(dims[1] - avg_dim),
    std::abs(dims[2] - avg_dim)
  });

  auto in_range = [tol](float val, float expected) -> float {
    float diff = std::abs(val - expected);
    if (diff > tol) return 0.0f;  // 超出容差
    return 1.0f - (diff / tol);   // [0, 1] 越接近越高的匹配度
  };

  // ── 敌方机器人 (700mm 立方体) ──
  {
    float score = (in_range(obs.length, enemy_robot_size_) +
                   in_range(obs.width,  enemy_robot_size_) +
                   in_range(obs.height, enemy_robot_size_)) / 3.0f;
    if (score > 0.5f) {
      obs.type = ObstacleType::ENEMY_ROBOT;
      obs.confidence = score;
      return;
    }
  }

  // ── 地球方块 (350mm 立方体) ──
  {
    float score = (in_range(obs.length, earth_cube_size_) +
                   in_range(obs.width,  earth_cube_size_) +
                   in_range(obs.height, earth_cube_size_)) / 3.0f;
    if (score > 0.5f) {
      obs.type = ObstacleType::EARTH_CUBE;
      obs.confidence = score;
      return;
    }
  }

  // ── 天空方块 (200mm 立方体) vs 穆斯蒂卡 (200mm 球体) ──
  // 两者尺寸相同, 通过 PCA 特征值区分:
  //   球体: 三特征值接近 (λ1≈λ2≈λ3)
  //   立方体: 特征值有一定差异
  {
    float size_score = (in_range(obs.length, sky_cube_size_) +
                        in_range(obs.width,  sky_cube_size_) +
                        in_range(obs.height, sky_cube_size_)) / 3.0f;

    if (size_score > 0.5f) {
      // 球体判断: 特征值比值都在 [1-tol, 1+tol] 范围内
      const bool is_spherical =
        obs.eigen_ratio_1 > 0.0f &&
        obs.eigen_ratio_2 > 0.0f &&
        std::abs(obs.eigen_ratio_1 - 1.0f) < sphere_eigen_tol_ &&
        std::abs(obs.eigen_ratio_2 - 1.0f) < sphere_eigen_tol_;

      if (is_spherical) {
        obs.type = ObstacleType::MUSTIKA;
        obs.confidence = size_score * 0.85f;  // 略低于立方体, 因为球体判断更不确定
      } else {
        obs.type = ObstacleType::SKY_CUBE;
        obs.confidence = size_score;
      }
      return;
    }
  }

  // ── 兜底: UNKNOWN ──
  obs.type = ObstacleType::UNKNOWN;
  obs.confidence = 0.0f;
}

// ═══════════════════════════════════════════════════════════════════════════
// 圆柱掩膜
//
// 将圆柱中心 (参数配置) 周围 cylinder_mask_radius_ 内的点移除.
// 用于避免圆柱残留点被误聚类为障碍物.
// ═══════════════════════════════════════════════════════════════════════════
ClusterExtractor::PointCloudPtr
ClusterExtractor::mask_cylinders(const PointCloudPtr& cloud)
{
  const size_t n_cylinders = cylinder_centers_.size();
  if (n_cylinders == 0) return cloud;

  const float r_sq = static_cast<float>(cylinder_mask_radius_ * cylinder_mask_radius_);

  auto filtered = std::make_shared<PointCloud>();
  filtered->reserve(cloud->size());

  size_t removed = 0;
  for (const auto& pt : cloud->points) {
    bool inside = false;
    for (size_t i = 0; i < n_cylinders; ++i) {
      float dx = pt.x - static_cast<float>(cylinder_centers_[i].first);
      float dy = pt.y - static_cast<float>(cylinder_centers_[i].second);
      if (dx * dx + dy * dy < r_sq) {
        inside = true;
        break;
      }
    }
    if (inside) {
      removed++;
    } else {
      filtered->push_back(pt);
    }
  }

  if (removed > 0) {
    RCLCPP_DEBUG(this->get_logger(),
      "Cylinder mask removed %zu/%zu points", removed, cloud->size());
  }

  return filtered;
}

// ═══════════════════════════════════════════════════════════════════════════
// 发布: MarkerArray (包围盒 + 文本标签)
// ═══════════════════════════════════════════════════════════════════════════
void ClusterExtractor::publish_obstacles(
    const std::vector<Obstacle>& obstacles,
    const std_msgs::msg::Header& header)
{
  visualization_msgs::msg::MarkerArray arr;

  for (const auto& obs : obstacles) {
    // ── 包围盒 CUBE ──
    {
      visualization_msgs::msg::Marker m;
      m.header = header;
      m.ns     = "obstacles";
      m.id     = obs.id;
      m.type   = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;

      // 中心位置
      m.pose.position.x = obs.centroid_x;
      m.pose.position.y = obs.centroid_y;
      m.pose.position.z = obs.centroid_z;

      // 方向 (绕 Z 轴旋转 yaw)
      m.pose.orientation.w = std::cos(obs.yaw * 0.5f);
      m.pose.orientation.x = 0.0;
      m.pose.orientation.y = 0.0;
      m.pose.orientation.z = std::sin(obs.yaw * 0.5f);

      // 尺寸 (使用包围盒尺寸)
      m.scale.x = std::max(obs.length, 0.02f);
      m.scale.y = std::max(obs.width,  0.02f);
      m.scale.z = std::max(obs.height, 0.02f);

      // 颜色 (按分类)
      switch (obs.type) {
        case ObstacleType::EARTH_CUBE:
          m.color.r = 0.16f; m.color.g = 0.39f; m.color.b = 0.20f; break;  // 深绿
        case ObstacleType::SKY_CUBE:
          m.color.r = 0.20f; m.color.g = 0.60f; m.color.b = 0.86f; break;  // 天蓝
        case ObstacleType::ENEMY_ROBOT:
          m.color.r = 0.85f; m.color.g = 0.20f; m.color.b = 0.20f; break;  // 红
        case ObstacleType::MUSTIKA:
          m.color.r = 0.85f; m.color.g = 0.65f; m.color.b = 0.13f; break;  // 金
        default:
          m.color.r = 0.5f; m.color.g = 0.5f; m.color.b = 0.5f; break;     // 灰
      }
      m.color.a = 0.5f;  // 半透明
      m.lifetime = rclcpp::Duration::from_seconds(1.0);
      arr.markers.push_back(m);
    }

    // ── 分类标签 TEXT ──
    {
      visualization_msgs::msg::Marker text;
      text.header = header;
      text.ns     = "obstacle_labels";
      text.id     = obs.id + 10000;
      text.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;

      text.pose.position.x = obs.centroid_x;
      text.pose.position.y = obs.centroid_y;
      text.pose.position.z = obs.max_z + 0.10f;  // 浮在顶部上方
      text.pose.orientation.w = 1.0;

      text.scale.z = 0.08f;  // 文字高度 8cm

      // 文本: 类型 + 置信度
      char buf[64];
      snprintf(buf, sizeof(buf), "%s (%.0f%%)",
               obstacle_type_name(obs.type), obs.confidence * 100.0f);
      text.text = buf;

      text.color.r = 1.0f; text.color.g = 1.0f; text.color.b = 1.0f;
      text.color.a = 1.0f;
      text.lifetime = rclcpp::Duration::from_seconds(1.0);
      arr.markers.push_back(text);
    }
  }

  // ── 清理过期 marker (DELETEALL 在无障碍物时清除) ──
  if (obstacles.empty()) {
    visualization_msgs::msg::Marker del;
    del.header = header;
    del.ns     = "obstacles";
    del.id     = 0;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    visualization_msgs::msg::Marker del2;
    del2.header = header;
    del2.ns     = "obstacle_labels";
    del2.id     = 0;
    del2.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del2);
  }

  pub_obstacles_->publish(arr);
}

// ═══════════════════════════════════════════════════════════════════════════
// 发布: Diagnostics
// ═══════════════════════════════════════════════════════════════════════════
void ClusterExtractor::publish_diagnostics(
    double total_ms,
    size_t cluster_count,
    size_t passed_count,
    const std::vector<Obstacle>& obstacles)
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "cluster_extractor";
  st.hardware_id = "mid360";

  if (total_ms < 15.0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "OK";
  } else if (total_ms < 25.0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "SLOW";
  } else {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.message = "VERY SLOW";
  }

  auto add = [&](const std::string& k, double v) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = k; kv.value = std::to_string(v);
    st.values.push_back(kv);
  };

  add("total_ms",       total_ms);
  add("cluster_count",  static_cast<double>(cluster_count));
  add("passed_count",   static_cast<double>(passed_count));
  add("filtered_count", static_cast<double>(cluster_count - passed_count));

  // 各类别计数
  std::unordered_map<ObstacleType, int> type_counts;
  for (const auto& obs : obstacles) {
    type_counts[obs.type]++;
  }
  add("earth_cube",   static_cast<double>(type_counts[ObstacleType::EARTH_CUBE]));
  add("sky_cube",     static_cast<double>(type_counts[ObstacleType::SKY_CUBE]));
  add("enemy_robot",  static_cast<double>(type_counts[ObstacleType::ENEMY_ROBOT]));
  add("mustika",      static_cast<double>(type_counts[ObstacleType::MUSTIKA]));
  add("unknown",      static_cast<double>(type_counts[ObstacleType::UNKNOWN]));

  arr.status.push_back(st);
  pub_diagnostics_->publish(arr);
}

// ═══════════════════════════════════════════════════════════════════════════
// 动态参数更新
// ═══════════════════════════════════════════════════════════════════════════
rcl_interfaces::msg::SetParametersResult ClusterExtractor::on_param_change(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool cylinder_centers_changed = false;

  for (const auto& p : params) {
    const std::string& name = p.get_name();

    try {
      #define SET(n, v)              if (name == n) v = p.as_double()
      #define SET_I(n, v)            if (name == n) v = p.as_int()
      #define SET_B(n, v)            if (name == n) v = p.as_bool()
      #define ARRAY_CHANGED(n)       if (name == n) cylinder_centers_changed = true

      SET("cluster_tolerance",    cluster_tolerance_);
      SET_I("min_cluster_size",   min_cluster_size_);
      SET_I("max_cluster_size",   max_cluster_size_);
      SET("max_object_size",      max_object_size_);
      SET("max_object_height",    max_object_height_);
      SET_I("min_object_points",  min_object_points_);
      SET("floating_z_min",       floating_z_min_);
      SET_B("enable_cylinder_mask", enable_cylinder_mask_);
      SET("cylinder_mask_radius", cylinder_mask_radius_);
      SET("size_tolerance",       size_tolerance_);
      SET("earth_cube_size",      earth_cube_size_);
      SET("sky_cube_size",        sky_cube_size_);
      SET("enemy_robot_size",     enemy_robot_size_);
      SET("mustika_size",         mustika_size_);
      SET("elongation_ratio",     elongation_ratio_);
      SET("sphere_eigen_tol",     sphere_eigen_tol_);

      ARRAY_CHANGED("cylinder_mask_x");
      ARRAY_CHANGED("cylinder_mask_y");

      #undef SET
      #undef SET_I
      #undef SET_B
      #undef ARRAY_CHANGED
    }
    catch (const rclcpp::ParameterTypeException& e) {
      result.successful = false;
      result.reason = std::string("Type mismatch: ") + name + " — " + e.what();
      break;
    }
  }

  // 圆柱中心数组变更 → 重新读取并重建 pair 向量
  if (cylinder_centers_changed && result.successful) {
    std::vector<double> cx, cy;
    this->get_parameter("cylinder_mask_x", cx);
    this->get_parameter("cylinder_mask_y", cy);
    if (cx.size() == cy.size()) {
      cylinder_centers_.clear();
      cylinder_centers_.reserve(cx.size());
      for (size_t i = 0; i < cx.size(); ++i) {
        cylinder_centers_.emplace_back(cx[i], cy[i]);
      }
    } else {
      result.successful = false;
      result.reason = "cylinder_mask_x and cylinder_mask_y must have same length";
    }
  }

  if (result.successful) {
    RCLCPP_DEBUG(this->get_logger(),
      "Params updated | tol=%.3f min=%d max=%d | "
      "filter: size=%.2f h=%.2f pts=%d floating=%.2f | "
      "classify: tol=%.3f earth=%.2f sky=%.2f robot=%.2f mustika=%.2f",
      cluster_tolerance_, min_cluster_size_, max_cluster_size_,
      max_object_size_, max_object_height_, min_object_points_, floating_z_min_,
      size_tolerance_, earth_cube_size_, sky_cube_size_,
      enemy_robot_size_, mustika_size_);
  } else {
    RCLCPP_WARN(this->get_logger(), "Param update rejected: %s", result.reason.c_str());
  }

  return result;
}

}  // namespace lidar
}  // namespace br_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::lidar::ClusterExtractor)
