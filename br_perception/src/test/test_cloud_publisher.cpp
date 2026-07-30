/**
 * @file test_cloud_publisher.cpp
 * @brief 模拟 Livox Mid-360 点云发布者 — 无雷达硬件时用于开发测试
 *
 * 生成:
 *   - 地面平面 (z=0, 8m×6m)
 *   - 2 根圆柱 (模拟 Mustika 柱 + Core 柱)
 *   - 4 个建筑位上的方块 (不同高度: 1块地球/2块地球/完整塔/空)
 *   - 少量随机噪点
 *
 * 发布话题: /livox/lidar/pointcloud (sensor_msgs::PointCloud2)
 *
 * 使用:
 *   ros2 run br_perception test_cloud_publisher --ros-args -p publish_rate:=10.0
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <random>
#include <cmath>
#include <chrono>

class TestCloudPublisher : public rclcpp::Node
{
public:
  TestCloudPublisher()
    : Node("test_cloud_publisher")
  {
    this->declare_parameter<double>("publish_rate", 10.0);
    this->declare_parameter<int>("num_ground_points", 5000);
    this->declare_parameter<int>("num_object_points", 200);
    this->get_parameter("publish_rate", publish_rate_);
    this->get_parameter("num_ground_points", num_ground_);
    this->get_parameter("num_object_points", num_object_);

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar/pointcloud", rclcpp::SystemDefaultsQoS());

    double period = 1.0 / publish_rate_;
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&TestCloudPublisher::publish, this));

    RCLCPP_INFO(this->get_logger(),
      "TestCloudPublisher ready | rate=%.1f Hz | ground=%d pts | objects=%d pts",
      publish_rate_, num_ground_, num_object_);
  }

private:
  void publish()
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.header.frame_id = "livox_frame";

    // ── 1. 地面平面 (8m×6m, z≈0, 带轻微噪声) ──
    const float ground_w = 8.0f, ground_h = 6.0f;
    const int g_per_dim = static_cast<int>(std::sqrt(static_cast<float>(num_ground_)));
    const float g_dx = ground_w / g_per_dim;
    const float g_dy = ground_h / g_per_dim;

    for (int i = 0; i < g_per_dim; ++i) {
      for (int j = 0; j < g_per_dim; ++j) {
        pcl::PointXYZ pt;
        pt.x = -1.0f + i * g_dx;
        pt.y = -3.0f + j * g_dy;
        pt.z = noise_01_(rng_) * 0.02f;  // ±2cm 地面起伏
        cloud.push_back(pt);
      }
    }

    // ── 2. Mustika 柱 (x=2.0, y=1.0, r=0.12, h=0.5) ──
    add_cylinder(cloud, 2.0f, 1.0f, 0.0f, 0.12f, 0.50f, num_object_);

    // ── 3. Core 柱 (x=5.0, y=1.0, r=0.15, h=0.8) ──
    add_cylinder(cloud, 5.0f, 1.0f, 0.0f, 0.15f, 0.80f, num_object_);

    // ── 4. 建筑位方块 (模拟不同高度的塔) ──
    // Spot 0: ONE_EARTH — 一块地球方块 (~0.35m)
    add_cube(cloud, 1.0f, 0.5f, 0.0f, 0.25f, 0.35f, num_object_ / 2);
    // Spot 1: TWO_EARTH — 两块地球方块 (~0.70m)
    add_cube(cloud, 1.0f, 2.0f, 0.0f, 0.25f, 0.70f, num_object_ / 2);
    // Spot 2: COMPLETE_TOWER — 完整塔 (~0.90m)
    add_cube(cloud, 4.0f, 0.5f, 0.0f, 0.25f, 0.90f, num_object_ / 2);
    // Spot 3: EMPTY (不添加任何点)
    // Spot 4: 一块地球方块 (另一个位置)
    add_cube(cloud, 7.0f, 2.0f, 0.0f, 0.25f, 0.35f, num_object_ / 2);
    // Spot 5: 敌方机器人 (700mm 立方体)
    add_cube(cloud, 8.5f, 1.0f, 0.0f, 0.35f, 0.70f, num_object_);

    // ── 5. 少量随机噪点 ──
    for (int i = 0; i < 200; ++i) {
      pcl::PointXYZ pt;
      pt.x = dist_01_(rng_) * 10.0f - 1.0f;
      pt.y = dist_01_(rng_) * 8.0f - 4.0f;
      pt.z = dist_01_(rng_) * 2.0f;
      cloud.push_back(pt);
    }

    // ── 发布 ──
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = this->now();
    msg.header.frame_id = "livox_frame";
    pub_->publish(msg);

    RCLCPP_DEBUG(this->get_logger(),
      "Published %zu points", cloud.size());
  }

  void add_cylinder(pcl::PointCloud<pcl::PointXYZ>& cloud,
                    float cx, float cy, float bz,
                    float radius, float height, int num_pts)
  {
    for (int i = 0; i < num_pts; ++i) {
      float angle = dist_01_(rng_) * 2.0f * static_cast<float>(M_PI);
      float r = radius * (0.9f + dist_01_(rng_) * 0.1f);  // ±10% 噪声
      float x = cx + r * std::cos(angle);
      float y = cy + r * std::sin(angle);
      float z = bz + dist_01_(rng_) * height;

      pcl::PointXYZ pt;
      pt.x = x; pt.y = y; pt.z = z;
      cloud.push_back(pt);
    }

    // 顶部加几个点 (模拟顶面)
    for (int i = 0; i < num_pts / 5; ++i) {
      float angle = dist_01_(rng_) * 2.0f * static_cast<float>(M_PI);
      float r = dist_01_(rng_) * radius * 0.8f;
      pcl::PointXYZ pt;
      pt.x = cx + r * std::cos(angle);
      pt.y = cy + r * std::sin(angle);
      pt.z = bz + height + noise_01_(rng_) * 0.01f;  // 顶面 ±1cm
      cloud.push_back(pt);
    }
  }

  void add_cube(pcl::PointCloud<pcl::PointXYZ>& cloud,
                float cx, float cy, float bz,
                float half_size, float height, int num_pts)
  {
    for (int i = 0; i < num_pts; ++i) {
      pcl::PointXYZ pt;
      pt.x = cx + (dist_01_(rng_) - 0.5f) * 2.0f * half_size;
      pt.y = cy + (dist_01_(rng_) - 0.5f) * 2.0f * half_size;
      pt.z = bz + dist_01_(rng_) * height;
      cloud.push_back(pt);
    }

    // 顶部加一层点 (模拟近水平顶面)
    for (int i = 0; i < num_pts / 3; ++i) {
      pcl::PointXYZ pt;
      pt.x = cx + (dist_01_(rng_) - 0.5f) * 2.0f * half_size;
      pt.y = cy + (dist_01_(rng_) - 0.5f) * 2.0f * half_size;
      pt.z = bz + height + noise_01_(rng_) * 0.005f;  // 顶面 ±5mm
      cloud.push_back(pt);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double publish_rate_{10.0};
  int    num_ground_{5000};
  int    num_object_{200};

  std::mt19937 rng_{42};
  std::uniform_real_distribution<float> dist_01_{0.0f, 1.0f};
  std::uniform_real_distribution<float> noise_01_{-1.0f, 1.0f};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
