#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <mutex>
#include <vector>

namespace br_perception {
namespace lidar {

/**
 * @brief 雷达点云预处理节点
 *
 * 管线顺序：
 *   原始点云 → VoxelGrid降采样 → 离群点过滤 → CropBox(XYZ联合裁剪) → [自过滤]
 *
 * 发布话题:
 *   /perception/lidar/filtered       (sensor_msgs::PointCloud2)
 *   /perception/lidar/diagnostics    (diagnostic_msgs::DiagnosticArray)
 */
class LidarPreprocessor : public rclcpp::Node
{
public:
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = PointCloud::Ptr;

  explicit LidarPreprocessor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ── ROS 回调 ──
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter>& params);

  // ── 滤波管线 ──
  PointCloudPtr filter_cloud(const PointCloudPtr& input);

  // ── 子步骤（可独立计时） ──
  PointCloudPtr downsample(const PointCloudPtr& input);
  PointCloudPtr remove_outliers(const PointCloudPtr& input);
  PointCloudPtr crop_roi(const PointCloudPtr& input);
  PointCloudPtr remove_self(const PointCloudPtr& input);

  // ── 诊断 ──
  void publish_diagnostics(
      double total_ms,
      double downsample_ms,
      double outlier_ms,
      double crop_ms,
      double self_ms,
      size_t input_points,
      size_t output_points);

  // ── 话题 ──
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_raw_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ── 参数 ──
  // 降采样
  double leaf_size_{0.05};          // 体素网格尺寸 (m)

  // 离群点过滤
  bool enable_outlier_filter_{true};
  int outlier_mean_k_{10};          // 邻域搜索点数
  double outlier_std_thresh_{1.0};  // 标准差倍数阈值

  // ROI 裁剪 (XYZ 联合)
  double roi_min_x_{-10.0};
  double roi_max_x_{30.0};
  double roi_min_y_{-10.0};
  double roi_max_y_{10.0};
  double roi_min_z_{-0.5};
  double roi_max_z_{2.0};

  // 自过滤
  bool enable_self_filter_{false};
  double self_radius_x_{0.35};      // 机器人半宽 (BR 700mm/2)
  double self_radius_y_{0.35};      // 机器人半深
  double self_radius_z_{0.7};       // 机器人半高 (BR 展开高度约 1200mm，保守取 700mm)

  // ── 状态 ──
  geometry_msgs::msg::PoseStamped current_pose_;
  std::mutex pose_mutex_;
  bool has_pose_{false};
};

}  // namespace lidar
}  // namespace br_perception
