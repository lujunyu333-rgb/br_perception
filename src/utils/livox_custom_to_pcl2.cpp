// ═══════════════════════════════════════════════════════════════════════════
// livox_custom_to_pcl2 — Livox CustomMsg → sensor_msgs/PointCloud2 桥接
//
// 为什么需要:
//   Point-LIO 要求驱动以 CustomMsg 输出 (xfer_format:=1), 而感知管线 (§3) 订阅的
//   是 PointCloud2。一个驱动实例只能出一种格式, 故用本节点做桥接:
//
//     livox_ros_driver2 ──/livox/lidar (CustomMsg)──┬─→ point_lio (里程计)
//                                                   └─→ 本节点 → /livox/lidar/pointcloud
//                                                                      ↓
//                                                          lidar_perception_node (§3)
//
// 输出保留 x/y/z + intensity(反射率); 逐点时间戳 (offset_time) 不在此桥接 —
// Point-LIO 直接用 CustomMsg, 感知管线不需要 per-point 时间。
// ═══════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>

namespace br_perception {
namespace utils {

class LivoxCustomToPcl2 : public rclcpp::Node
{
public:
  LivoxCustomToPcl2() : Node("livox_custom_to_pcl2")
  {
    input_topic_  = this->declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/livox/lidar/pointcloud");
    frame_id_     = this->declare_parameter<std::string>("frame_id", "livox_frame");

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_, rclcpp::SystemDefaultsQoS());
    // ⚠ 订阅端用 BEST_EFFORT: 对 RELIABLE 与 BEST_EFFORT 两种发布端都兼容,
    //   避免驱动侧 QoS 变化时静默收不到 (发布端保持 RELIABLE — 那对两种订阅者都兼容)
    sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LivoxCustomToPcl2::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "CustomMsg → PointCloud2: %s → %s (frame=%s)",
                input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str());
  }

private:
  void cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(msg->point_num);
    for (const auto& p : msg->points) {
      pcl::PointXYZI q;
      q.x = p.x;
      q.y = p.y;
      q.z = p.z;
      q.intensity = static_cast<float>(p.reflectivity);
      cloud.push_back(std::move(q));
    }

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(cloud, out);
    out.header.stamp    = msg->header.stamp;
    out.header.frame_id = frame_id_;
    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace utils
}  // namespace br_perception

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<br_perception::utils::LivoxCustomToPcl2>());
  rclcpp::shutdown();
  return 0;
}
