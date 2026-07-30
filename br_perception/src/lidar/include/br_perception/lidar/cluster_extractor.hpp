#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <memory>
#include <vector>
#include <string>
#include <utility>      // std::pair
#include <cstdint>

namespace br_perception {
namespace lidar {

// ── 障碍物分类 ──
enum class ObstacleType : uint8_t {
  UNKNOWN      = 0,
  EARTH_CUBE   = 1,   // ≈350mm 立方体 — 地球方块
  SKY_CUBE     = 2,   // ≈200mm 立方体 — 天空方块
  ENEMY_ROBOT  = 3,   // ≈700×700×700mm — 敌方机器人
  MUSTIKA      = 4    // ≈200mm 球体 — 穆斯蒂卡
};

// ── 障碍物检测结果 ──
struct Obstacle {
  int id = 0;

  // 3D 包围盒
  float min_x = 0, min_y = 0, min_z = 0;
  float max_x = 0, max_y = 0, max_z = 0;

  // 质心
  float centroid_x = 0, centroid_y = 0, centroid_z = 0;

  // 尺寸 (长/宽/高)
  float length = 0, width = 0, height = 0;

  // 统计
  int   point_count = 0;

  // 方向 (PCA 主轴水平投影偏航角, rad)
  float yaw = 0;
  bool  is_elongated = false;    // 长宽比 > threshold 时为 true

  // 分类
  ObstacleType type = ObstacleType::UNKNOWN;
  float confidence = 0;          // [0-1]

  // PCA 特征值 (用于球体/立方体区分)
  float eigen_ratio_1 = 0;       // λ1 / λ2
  float eigen_ratio_2 = 0;       // λ2 / λ3
};

/**
 * @brief 障碍物聚类与分类节点 — ROBOCON 2027 适配版
 *
 * 管线:
 *   非地面点云(去圆柱) → [圆柱掩膜] → 欧式聚类 →
 *   特征计算(包围盒/PCA/质心) → 规则过滤 → 尺寸分类 → 发布
 *
 * 聚类参数从 lidar_params.yaml 读取 (ROS2 parameter 机制).
 *
 * 输入:
 *   /perception/lidar/non_ground     — 非地面点云 (sensor_msgs::PointCloud2)
 *
 * 输出:
 *   /perception/lidar/obstacles      — 3D 包围盒 + 分类标签 (visualization_msgs::MarkerArray)
 *   /perception/lidar/diagnostics     — 耗时 + 统计 (diagnostic_msgs::DiagnosticArray)
 */
class ClusterExtractor : public rclcpp::Node
{
public:
  using PointCloud    = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = PointCloud::Ptr;

  explicit ClusterExtractor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ── ROS 回调 ──
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_param_change(
      const std::vector<rclcpp::Parameter>& params);

  // ── 核心管线 ──
  /// 欧式聚类, 返回聚类索引列表
  std::vector<pcl::PointIndices> extract_clusters(const PointCloudPtr& cloud);

  /// 对单个聚类计算所有特征 (包围盒, 质心, PCA, 点数)
  Obstacle compute_obstacle(const PointCloudPtr& cloud,
                            const pcl::PointIndices& cluster,
                            int id);

  /// 3D PCA: 计算主轴方向 + 判断是否 elongated
  void compute_pca_orientation(const PointCloudPtr& cloud,
                               const pcl::PointIndices& cluster,
                               Obstacle& obs);

  // ── 过滤 ──
  /// @return true 如果该障碍物应被剔除
  bool filter_obstacle(const Obstacle& obs) const;

  // ── 分类 ──
  /// 基于包围盒尺寸 + PCA 特征对障碍物进行分类猜测
  void classify_obstacle(Obstacle& obs);

  // ── 圆柱掩膜 ──
  /// 移除已知圆柱周围区域的点, 避免圆柱残点被误聚类
  PointCloudPtr mask_cylinders(const PointCloudPtr& cloud);

  // ── 发布 ──
  void publish_obstacles(const std::vector<Obstacle>& obstacles,
                          const std_msgs::msg::Header& header);
  void publish_diagnostics(double total_ms,
                           size_t cluster_count,
                           size_t passed_count,
                           const std::vector<Obstacle>& obstacles);

  // ── 话题 ──
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacles_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ══════════════════════════════════════════════════════════════
  // 聚类参数
  // ══════════════════════════════════════════════════════════════
  double cluster_tolerance_{0.05};       // 欧式聚类距离阈值 (m)
  int    min_cluster_size_{5};           // 最小聚类点数
  int    max_cluster_size_{5000};        // 最大聚类点数

  // ══════════════════════════════════════════════════════════════
  // 过滤参数 (ROBOCON 2027 场景)
  // ══════════════════════════════════════════════════════════════
  double max_object_size_{2.0};          // 任一维度超此值 → 围栏/墙面 (m)
  double max_object_height_{2.0};        // 高度超此值 → 柱子 (m)
  int    min_object_points_{5};          // 点数少于此 → 噪点
  double floating_z_min_{0.8};           // 底部 z 高于此值 → 悬空物 (m)

  // ══════════════════════════════════════════════════════════════
  // 圆柱掩膜参数
  // ══════════════════════════════════════════════════════════════
  bool   enable_cylinder_mask_{true};
  double cylinder_mask_radius_{0.25};    // 圆柱外扩半径 (m)
  // 圆柱中心列表 (从 YAML 加载), {x, y} 对, 避免两个独立数组越界
  std::vector<std::pair<double, double>> cylinder_centers_;

  // ══════════════════════════════════════════════════════════════
  // 分类参数
  // ══════════════════════════════════════════════════════════════
  double size_tolerance_{0.08};          // 分类尺寸容差 (m)
  double earth_cube_size_{0.35};         // 地球方块预期尺寸 (m)
  double sky_cube_size_{0.20};           // 天空方块预期尺寸 (m)
  double enemy_robot_size_{0.70};        // 敌方机器人预期尺寸 (m)
  double mustika_size_{0.20};            // 穆斯蒂卡预期直径 (m)
  double elongation_ratio_{2.0};         // PCA λ1/λ2 超过此值 → elongated
  double sphere_eigen_tol_{0.3};         // 球体判断: 特征值差异容差
};

}  // namespace lidar
}  // namespace br_perception
