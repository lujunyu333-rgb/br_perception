#include "br_perception/lidar/lidar_preprocessor.hpp"

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <string>
#include <algorithm>

namespace br_perception {
namespace lidar {

// ============================================================================
// 构造 & 参数初始化
// ============================================================================
LidarPreprocessor::LidarPreprocessor(const rclcpp::NodeOptions& options)
  : Node("lidar_preprocessor", options)
{
  // 声明所有参数
  this->declare_parameter<double>("leaf_size", leaf_size_);
  this->declare_parameter<bool>("enable_outlier_filter", enable_outlier_filter_);
  this->declare_parameter<int>("outlier_mean_k", outlier_mean_k_);
  this->declare_parameter<double>("outlier_std_thresh", outlier_std_thresh_);
  this->declare_parameter<double>("roi_min_x", roi_min_x_);
  this->declare_parameter<double>("roi_max_x", roi_max_x_);
  this->declare_parameter<double>("roi_min_y", roi_min_y_);
  this->declare_parameter<double>("roi_max_y", roi_max_y_);
  this->declare_parameter<double>("roi_min_z", roi_min_z_);
  this->declare_parameter<double>("roi_max_z", roi_max_z_);
  this->declare_parameter<bool>("enable_self_filter", enable_self_filter_);
  this->declare_parameter<double>("self_radius_x", self_radius_x_);
  this->declare_parameter<double>("self_radius_y", self_radius_y_);
  this->declare_parameter<double>("self_radius_z", self_radius_z_);

  // 读取参数
  this->get_parameter("leaf_size", leaf_size_);
  this->get_parameter("enable_outlier_filter", enable_outlier_filter_);
  this->get_parameter("outlier_mean_k", outlier_mean_k_);
  this->get_parameter("outlier_std_thresh", outlier_std_thresh_);
  this->get_parameter("roi_min_x", roi_min_x_);
  this->get_parameter("roi_max_x", roi_max_x_);
  this->get_parameter("roi_min_y", roi_min_y_);
  this->get_parameter("roi_max_y", roi_max_y_);
  this->get_parameter("roi_min_z", roi_min_z_);
  this->get_parameter("roi_max_z", roi_max_z_);
  this->get_parameter("enable_self_filter", enable_self_filter_);
  this->get_parameter("self_radius_x", self_radius_x_);
  this->get_parameter("self_radius_y", self_radius_y_);
  this->get_parameter("self_radius_z", self_radius_z_);

  // 参数动态更新回调
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&LidarPreprocessor::on_parameter_change, this, std::placeholders::_1));

  // 订阅原始点云 (best-effort QoS，不阻塞发送端)
  sub_raw_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar/pointcloud",
    rclcpp::SensorDataQoS(),
    std::bind(&LidarPreprocessor::pointcloud_callback, this, std::placeholders::_1));

  // 发布滤波后点云
  pub_filtered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/perception/lidar/filtered",
    rclcpp::SensorDataQoS());

  // 发布诊断信息
  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/perception/lidar/diagnostics", 10);

  // 自过滤：订阅机器人位姿
  if (enable_self_filter_) {
    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/robot_pose", 10,
      std::bind(&LidarPreprocessor::pose_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Self-filter ENABLED");
  }

  RCLCPP_INFO(this->get_logger(),
    "LidarPreprocessor ready | leaf=%.3fm | outlier=%s(k=%d,σ=%.1f) | "
    "roi_x=[%.1f,%.1f] roi_y=[%.1f,%.1f] roi_z=[%.1f,%.1f]",
    leaf_size_,
    enable_outlier_filter_ ? "on" : "off",
    outlier_mean_k_, outlier_std_thresh_,
    roi_min_x_, roi_max_x_,
    roi_min_y_, roi_max_y_,
    roi_min_z_, roi_max_z_);
}

// ============================================================================
// 点云回调
// ============================================================================
void LidarPreprocessor::pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // ── 解包 ──
  PointCloudPtr cloud_in = std::make_shared<PointCloud>();
  pcl::fromROSMsg(*msg, *cloud_in);

  const size_t input_points = cloud_in->size();
  if (input_points == 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                          5000, "Empty point cloud received");
    return;
  }

  // ── 滤波管线（每步独立计时） ──
  auto t0 = std::chrono::steady_clock::now();

  auto cloud_ds    = downsample(cloud_in);
  auto t1 = std::chrono::steady_clock::now();

  auto cloud_clean = remove_outliers(cloud_ds);
  auto t2 = std::chrono::steady_clock::now();

  auto cloud_crop  = crop_roi(cloud_clean);
  auto t3 = std::chrono::steady_clock::now();

  auto cloud_final = remove_self(cloud_crop);
  auto t4 = std::chrono::steady_clock::now();

  // ── 耗时统计 ──
  using ms = std::chrono::duration<float, std::milli>;
  double total_ms     = ms(t4 - t0).count();
  double downsample_ms= ms(t1 - t0).count();
  double outlier_ms   = ms(t2 - t1).count();
  double crop_ms      = ms(t3 - t2).count();
  double self_ms      = ms(t4 - t3).count();

  // ── 发布滤波结果 ──
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*cloud_final, output_msg);
  output_msg.header = msg->header;
  pub_filtered_->publish(output_msg);

  // ── 发布诊断 ──
  publish_diagnostics(total_ms, downsample_ms, outlier_ms,
                      crop_ms, self_ms, input_points, cloud_final->size());

  // 延迟超标告警
  if (total_ms > 20.0f) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Preprocessing slow: %.1f ms (input=%zu, output=%zu)",
      total_ms, input_points, cloud_final->size());
  }
}

// ============================================================================
// 滤波管线
// ============================================================================
LidarPreprocessor::PointCloudPtr LidarPreprocessor::filter_cloud(const PointCloudPtr& input)
{
  auto out = downsample(input);
  out = remove_outliers(out);
  out = crop_roi(out);
  out = remove_self(out);
  return out;
}

// ── 1. 降采样 ──
LidarPreprocessor::PointCloudPtr LidarPreprocessor::downsample(const PointCloudPtr& input)
{
  auto output = std::make_shared<PointCloud>();
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(input);
  voxel.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel.filter(*output);
  return output;
}

// ── 2. 离群点过滤 ──
LidarPreprocessor::PointCloudPtr LidarPreprocessor::remove_outliers(const PointCloudPtr& input)
{
  if (!enable_outlier_filter_ || input->size() < static_cast<size_t>(outlier_mean_k_)) {
    return input;
  }

  auto output = std::make_shared<PointCloud>();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(input);
  sor.setMeanK(outlier_mean_k_);
  sor.setStddevMulThresh(outlier_std_thresh_);
  sor.setNegative(false);  // false = 保留内点, 剔除离群点
  sor.filter(*output);
  return output;
}

// ── 3. ROI 裁剪（XYZ 联合 CropBox，替代分散的 PassThrough×2 + CropBox） ──
LidarPreprocessor::PointCloudPtr LidarPreprocessor::crop_roi(const PointCloudPtr& input)
{
  auto output = std::make_shared<PointCloud>();

  pcl::CropBox<pcl::PointXYZ> box;
  box.setInputCloud(input);
  box.setMin(Eigen::Vector4f(roi_min_x_, roi_min_y_, roi_min_z_, 1.0f));
  box.setMax(Eigen::Vector4f(roi_max_x_, roi_max_y_, roi_max_z_, 1.0f));
  box.setNegative(false);  // 保留内部
  box.filter(*output);
  return output;
}

// ── 4. 自过滤（线程安全） ──
LidarPreprocessor::PointCloudPtr LidarPreprocessor::remove_self(const PointCloudPtr& input)
{
  if (!enable_self_filter_) {
    return input;
  }

  // 读 current_pose_ 时加锁（pose_callback 在另一线程写入）
  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!has_pose_) {
      // 还没收到过位姿，跳过自过滤
      return input;
    }
    pose = current_pose_;
  }

  auto output = std::make_shared<PointCloud>();

  pcl::CropBox<pcl::PointXYZ> self_box;
  self_box.setInputCloud(input);
  self_box.setMin(Eigen::Vector4f(
    pose.pose.position.x - self_radius_x_,
    pose.pose.position.y - self_radius_y_,
    pose.pose.position.z - self_radius_z_,
    1.0f));
  self_box.setMax(Eigen::Vector4f(
    pose.pose.position.x + self_radius_x_,
    pose.pose.position.y + self_radius_y_,
    pose.pose.position.z + self_radius_z_,
    1.0f));
  self_box.setNegative(true);   // 剔除内部
  self_box.filter(*output);
  return output;
}

// ============================================================================
// 诊断
// ============================================================================
void LidarPreprocessor::publish_diagnostics(
    double total_ms,
    double downsample_ms,
    double outlier_ms,
    double crop_ms,
    double self_ms,
    size_t input_points,
    size_t output_points)
{
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "lidar_preprocessor";
  status.hardware_id = "mid360";

  // 总体健康判定
  if (total_ms < 15.0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";
  } else if (total_ms < 25.0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "SLOW";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "VERY SLOW";
  }

  // 各步骤耗时 + 点数信息 (key-value 对)
  auto add_kv = [&](const std::string& key, double val) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = std::to_string(val);
    status.values.push_back(kv);
  };

  add_kv("total_ms",       total_ms);
  add_kv("downsample_ms",  downsample_ms);
  add_kv("outlier_ms",     outlier_ms);
  add_kv("crop_ms",        crop_ms);
  add_kv("self_ms",        self_ms);
  add_kv("input_points",   static_cast<double>(input_points));
  add_kv("output_points",  static_cast<double>(output_points));
  add_kv("reduction_ratio",
         (input_points > 0) ? (1.0 - static_cast<double>(output_points) / input_points) : 0.0);
  add_kv("leaf_size",      leaf_size_);

  diag_array.status.push_back(status);
  pub_diagnostics_->publish(diag_array);
}

// ============================================================================
// 动态参数更新
// ============================================================================
rcl_interfaces::msg::SetParametersResult LidarPreprocessor::on_parameter_change(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;   // 默认成功
  result.reason = "";

  for (const auto& param : params) {
    const std::string& name = param.get_name();

    try {
      if (name == "leaf_size") {
        double val = param.as_double();
        if (val <= 0.0 || val > 1.0) {
          result.successful = false;
          result.reason = "leaf_size must be in (0, 1.0]";
          break;
        }
        leaf_size_ = val;
      }
      else if (name == "enable_outlier_filter") {
        enable_outlier_filter_ = param.as_bool();
      }
      else if (name == "outlier_mean_k") {
        int val = param.as_int();
        if (val < 3) {
          result.successful = false;
          result.reason = "outlier_mean_k must be >= 3";
          break;
        }
        outlier_mean_k_ = val;
      }
      else if (name == "outlier_std_thresh") {
        double val = param.as_double();
        if (val <= 0.0) {
          result.successful = false;
          result.reason = "outlier_std_thresh must be > 0";
          break;
        }
        outlier_std_thresh_ = val;
      }
      else if (name == "roi_min_x")  { roi_min_x_ = param.as_double(); }
      else if (name == "roi_max_x")  { roi_max_x_ = param.as_double(); }
      else if (name == "roi_min_y")  { roi_min_y_ = param.as_double(); }
      else if (name == "roi_max_y")  { roi_max_y_ = param.as_double(); }
      else if (name == "roi_min_z")  { roi_min_z_ = param.as_double(); }
      else if (name == "roi_max_z")  { roi_max_z_ = param.as_double(); }
      else if (name == "enable_self_filter") {
        enable_self_filter_ = param.as_bool();
        if (enable_self_filter_ && !sub_pose_) {
          sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_pose", 10,
            std::bind(&LidarPreprocessor::pose_callback, this, std::placeholders::_1));
        }
      }
      else if (name == "self_radius_x")  { self_radius_x_ = param.as_double(); }
      else if (name == "self_radius_y")  { self_radius_y_ = param.as_double(); }
      else if (name == "self_radius_z")  { self_radius_z_ = param.as_double(); }
      else {
        // 未知参数（可能属于其他节点），不标记失败，忽略即可
        RCLCPP_DEBUG(this->get_logger(), "Ignoring unknown param: %s", name.c_str());
      }
    }
    catch (const rclcpp::ParameterTypeException& e) {
      result.successful = false;
      result.reason = std::string("Type mismatch for ") + name + ": " + e.what();
      break;
    }
  }

  if (result.successful) {
    RCLCPP_INFO(this->get_logger(),
      "Params updated | leaf=%.3f outlier=%s(k=%d,σ=%.1f) self=%s",
      leaf_size_, enable_outlier_filter_ ? "on" : "off",
      outlier_mean_k_, outlier_std_thresh_,
      enable_self_filter_ ? "on" : "off");
  } else {
    RCLCPP_WARN(this->get_logger(), "Param update rejected: %s", result.reason.c_str());
  }

  return result;
}

// ============================================================================
// 位姿回调（自过滤用，在独立线程中调用）
// ============================================================================
void LidarPreprocessor::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  current_pose_ = *pose_msg;
  has_pose_ = true;
}

}  // namespace lidar
}  // namespace br_perception

// ROS2 组件注册
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::lidar::LidarPreprocessor)
