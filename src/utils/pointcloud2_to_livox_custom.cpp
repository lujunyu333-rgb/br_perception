// ═══════════════════════════════════════════════════════════════════════════
// pointcloud2_to_livox_custom — PointCloud2 → Livox CustomMsg 桥接 (Gazebo 仿真用)
//
// 为什么需要:
//   Gazebo 的 ray 传感器只能出 PointCloud2, 而 Point-LIO 的 Livox 通路
//   (lidar_type:=1, 已在实机验证) 要 CustomMsg。本节点把仿真点云"伪装"成
//   Mid-360 的 CustomMsg, 这样仿真与实机走同一条 Point-LIO 配置路径。
//
// 必须合成的字段 (见 lio_ws point_lio src/preprocess.cpp):
//   line        — 预处理过滤 `line < N_SCANS`, 且去畸变按 line 分桶 → 由仰角分桶
//   offset_time — → curvature, 去畸变用 (ns, 帧内偏移) → 由方位角映射到帧周期
//   tag         — 过滤 `(tag & 0x30) ∈ {0x00, 0x10}` → 固定 0
//   reflectivity— → intensity → 取 PointCloud2 的 intensity
//
// 用法 (仿真):
//   /livox/lidar/pointcloud (PCL2, Gazebo) → 本节点 → /livox/lidar (CustomMsg) → Point-LIO
// ═══════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace br_perception {
namespace utils {

class PointCloud2ToLivoxCustom : public rclcpp::Node
{
public:
  PointCloud2ToLivoxCustom() : Node("pointcloud2_to_livox_custom")
  {
    input_topic_   = this->declare_parameter<std::string>("input_topic", "/livox/lidar/pointcloud");
    output_topic_  = this->declare_parameter<std::string>("output_topic", "/livox/lidar");
    scan_lines_    = this->declare_parameter<int>("scan_lines", 16);
    frame_rate_hz_ = this->declare_parameter<double>("frame_rate_hz", 10.0);
    frame_id_      = this->declare_parameter<std::string>("frame_id", "livox_frame");
    // line 分桶用的仰角范围 (度)。默认 = Livox Mid-360 规格 (-7° ~ +52°)。
    // ⚠ 必须与输入雷达的垂直 FOV 一致: 当前 sensor_rig 是 ±14.9° (0.26rad),
    //   与实机不同 → bringup_gazebo.launch.py 里显式传 -14.9 / 14.9。
    elev_min_deg_  = this->declare_parameter<double>("elev_min_deg", -7.0);
    elev_max_deg_  = this->declare_parameter<double>("elev_max_deg", 52.0);

    pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
        output_topic_, rclcpp::SystemDefaultsQoS());
    // ⚠ 订阅端 BEST_EFFORT: Gazebo ray 插件按 sensor 数据发 BEST_EFFORT,
    //   RELIABLE 订阅者会一帧都收不到 (静默)
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PointCloud2ToLivoxCustom::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "PointCloud2 → CustomMsg: %s → %s (scan_lines=%d, %.1fHz, frame=%s)",
                input_topic_.c_str(), output_topic_.c_str(),
                scan_lines_, frame_rate_hz_, frame_id_.c_str());
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.empty()) {
      return;
    }

    // ── 仰角范围 (用于 line 分桶) ──
    // ⚠ 用配置的**固定**仰角范围, 不用每帧数据的 min/max:
    //   逐帧统计会让桶边界随场景内容漂移 (同一方向的点在不同帧落进不同 line),
    //   Point-LIO 按 line 分桶的去畸变会跟着抖。
    //   范围由 elev_min_deg / elev_max_deg 给出, 须与输入雷达的垂直 FOV 一致。
    const double el_min  = elev_min_deg_ * M_PI / 180.0;
    const double el_span = std::max((elev_max_deg_ - elev_min_deg_) * M_PI / 180.0, 1e-6);
    const double frame_period_ns = 1e9 / std::max(frame_rate_hz_, 1e-6);

    livox_ros_driver2::msg::CustomMsg out;
    out.header.stamp    = msg->header.stamp;
    out.header.frame_id = frame_id_;
    out.timebase        = 0;
    out.lidar_id        = 0;
    out.point_num       = static_cast<uint32_t>(cloud.size());
    out.points.resize(cloud.size());

    for (size_t i = 0; i < cloud.size(); ++i) {
      const auto& src = cloud.points[i];
      auto& dst = out.points[i];
      dst.x = src.x;
      dst.y = src.y;
      dst.z = src.z;
      dst.reflectivity = static_cast<uint8_t>(std::clamp(src.intensity, 0.0f, 255.0f));
      dst.tag = 0;   // 过滤条件 (tag & 0x30) ∈ {0x00, 0x10} 恒满足

      // line: 仰角分桶 0..scan_lines-1 (必须 < N_SCANS, 否则被预处理丢弃)
      const double r_xy = std::hypot(src.x, src.y);
      const double el = std::atan2(src.z, r_xy);
      int line = static_cast<int>((el - el_min) / el_span * scan_lines_);
      dst.line = static_cast<uint8_t>(std::clamp(line, 0, scan_lines_ - 1));

      // offset_time: 方位角 → 帧内时间 (模拟旋转扫描的逐点时序, 供去畸变)
      // 假设一帧从 -π 扫到 +π; 与 Mid-360 驱动一致, 单位 ns
      const double az = std::atan2(src.y, src.x);          // [-π, π)
      dst.offset_time = static_cast<uint32_t>((az + M_PI) / (2.0 * M_PI) * frame_period_ns);
    }

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  int scan_lines_;
  double frame_rate_hz_;
  double elev_min_deg_;
  double elev_max_deg_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_;
};

}  // namespace utils
}  // namespace br_perception

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<br_perception::utils::PointCloud2ToLivoxCustom>());
  rclcpp::shutdown();
  return 0;
}
