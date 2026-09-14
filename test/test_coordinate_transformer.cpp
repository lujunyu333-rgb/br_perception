// ═══════════════════════════════════════════════════════════════════════════
// test_coordinate_transformer — 坐标变换精度与里程计可靠性 (§11)
//
// 覆盖:
//   1. 里程计可靠性状态机: TIMEOUT / OK / DEGENERATE / IMU_VIBRATION
//   2. 置信度降权: 可靠 ×1.0, 退化 ×confidence_decay
//   3. BR_RESET 位姿注入: odom→world 校正的数学正确性
//   4. 相对定位模式: 退化时的局部坐标差
//   5. TF 不可用时的失败路径 (标记不可靠而非崩溃)
// ═══════════════════════════════════════════════════════════════════════════

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

#include <br_perception/fusion/coordinate_transformer.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
using br_perception::fusion::CoordinateTransformer;
using br_perception::fusion::OdometryState;
using br_perception::fusion::TransformerParams;

namespace {

// 测试专用话题 — 避免干扰真实系统
constexpr char kOdomTopic[]  = "/test_coord_tf/odom";
constexpr char kImuTopic[]   = "/test_coord_tf/imu";
constexpr char kDegenTopic[] = "/test_coord_tf/degenerate";

constexpr double kPi = 3.14159265358979323846;

class CoordinateTransformerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_coordinate_transformer");

    params_.odometry_topic   = kOdomTopic;
    params_.imu_topic        = kImuTopic;
    params_.degenerate_topic = kDegenTopic;
    params_.odom_timeout_sec        = 2.0;    // 默认放宽, 超时用例单独覆盖
    params_.imu_acc_variance_thresh = 0.5;    // 便于单测触发
    params_.confidence_decay        = 0.5;
    params_.tf_publish_rate_hz      = 0.0;    // 单测不限频

    transformer_ = std::make_unique<CoordinateTransformer>(node_.get(), params_);
    pub_odom_  = node_->create_publisher<nav_msgs::msg::Odometry>(kOdomTopic, 10);
    pub_imu_   = node_->create_publisher<sensor_msgs::msg::Imu>(kImuTopic, 50);
    pub_degen_ = node_->create_publisher<std_msgs::msg::Bool>(kDegenTopic, 10);
  }

  void TearDown() override
  {
    transformer_.reset();
    pub_odom_.reset();
    pub_imu_.reset();
    pub_degen_.reset();
    node_.reset();
  }

  /// 自旋等待, 让订阅回调处理已发布的消息
  void spin_for(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(2ms);
    }
  }

  void publish_odom(double x, double y, double yaw)
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp    = node_->now();
    msg.header.frame_id = "camera_init";
    msg.child_frame_id  = "aft_mapped";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    const double half = yaw * 0.5;
    msg.pose.pose.orientation.z = std::sin(half);
    msg.pose.pose.orientation.w = std::cos(half);
    pub_odom_->publish(msg);
    spin_for(120ms);
  }

  void publish_imu(double acc_norm_g)
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = node_->now();
    // 交替正负 → 制造大方差
    static bool flip = false;
    flip = !flip;
    msg.linear_acceleration.z = (flip ? 1.0 : -1.0) * acc_norm_g;
    pub_imu_->publish(msg);
  }

  std::shared_ptr<rclcpp::Node> node_;
  TransformerParams params_;
  std::unique_ptr<CoordinateTransformer> transformer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr    pub_imu_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      pub_degen_;
};

// ── 1. 状态机 ──────────────────────────────────────────────────────────────

TEST_F(CoordinateTransformerTest, 无里程计时判为超时且降权)
{
  EXPECT_EQ(OdometryState::TIMEOUT, transformer_->odometry_state());
  EXPECT_FALSE(transformer_->odometry_reliable());
  EXPECT_FLOAT_EQ(0.5f, transformer_->confidence_scale());
  EXPECT_FALSE(transformer_->robot_pose_world().has_value());
}

TEST_F(CoordinateTransformerTest, 收到里程计后判为可靠且不降权)
{
  publish_odom(1.0, 2.0, 0.0);
  EXPECT_EQ(OdometryState::OK, transformer_->odometry_state());
  EXPECT_TRUE(transformer_->odometry_reliable());
  EXPECT_FLOAT_EQ(1.0f, transformer_->confidence_scale());

  const auto pose = transformer_->robot_pose_world();
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(1.0, pose->x, 1e-6);
  EXPECT_NEAR(2.0, pose->y, 1e-6);
}

TEST_F(CoordinateTransformerTest, 里程计超时后降级)
{
  params_.odom_timeout_sec = 0.2;
  transformer_ = std::make_unique<CoordinateTransformer>(node_.get(), params_);

  publish_odom(0.5, 0.0, 0.0);
  ASSERT_TRUE(transformer_->odometry_reliable());

  spin_for(400ms);   // 不再发消息 → 超时
  EXPECT_EQ(OdometryState::TIMEOUT, transformer_->odometry_state());
  EXPECT_FLOAT_EQ(0.5f, transformer_->confidence_scale());
}

TEST_F(CoordinateTransformerTest, 退化标志触发降级)
{
  publish_odom(0.0, 0.0, 0.0);
  ASSERT_TRUE(transformer_->odometry_reliable());

  std_msgs::msg::Bool flag;
  flag.data = true;
  pub_degen_->publish(flag);
  spin_for(150ms);

  EXPECT_EQ(OdometryState::DEGENERATE, transformer_->odometry_state());
  EXPECT_FLOAT_EQ(0.5f, transformer_->confidence_scale());
  EXPECT_TRUE(transformer_->relative_mode());
}

TEST_F(CoordinateTransformerTest, IMU方差过大触发振动降级)
{
  publish_odom(0.0, 0.0, 0.0);
  ASSERT_TRUE(transformer_->odometry_reliable());

  // 加速度模长在 1g / 4g 交替 → 方差 2.25 g², 远超 0.5 阈值
  // (注意: 判定用模长方差, 仅翻转符号不会产生方差)
  for (int i = 0; i < 30; ++i) {
    publish_imu(i % 2 == 0 ? 1.0 : 4.0);
    spin_for(10ms);
  }
  EXPECT_EQ(OdometryState::IMU_VIBRATION, transformer_->odometry_state());
}

// ── 2. BR_RESET 位姿注入 (§4.1 v1.6) ───────────────────────────────────────

TEST_F(CoordinateTransformerTest, 位姿注入后world位姿对齐场地先验)
{
  // 里程计原点 → 注入 (1.0, 2.0, 90°)
  publish_odom(0.0, 0.0, 0.0);
  ASSERT_TRUE(transformer_->inject_pose_prior(1.0, 2.0, kPi / 2.0));
  EXPECT_TRUE(transformer_->pose_prior_active());

  auto pose = transformer_->robot_pose_world();
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(1.0, pose->x, 1e-6);
  EXPECT_NEAR(2.0, pose->y, 1e-6);
  EXPECT_NEAR(kPi / 2.0, pose->theta, 1e-6);

  // 之后沿 odom 局部 +x 前进 1m → world 应转到 +y 方向
  publish_odom(1.0, 0.0, 0.0);
  pose = transformer_->robot_pose_world();
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(1.0, pose->x, 1e-6);
  EXPECT_NEAR(3.0, pose->y, 1e-6);
}

TEST_F(CoordinateTransformerTest, 无里程计时位姿注入被拒)
{
  EXPECT_FALSE(transformer_->inject_pose_prior(1.0, 2.0, 0.0));
  EXPECT_FALSE(transformer_->pose_prior_active());
}

// ── 3. 相对定位模式 (v1.5) ─────────────────────────────────────────────────

TEST_F(CoordinateTransformerTest, 退化时提供局部坐标差)
{
  publish_odom(0.0, 0.0, 0.0);   // 最后可靠位姿 = 原点
  ASSERT_TRUE(transformer_->odometry_reliable());

  // 仍可靠时: 局部差恒为 0 (基准即当前最后可靠位姿, 会随可靠观测更新)
  ASSERT_TRUE(transformer_->relative_pose_delta().has_value());
  EXPECT_NEAR(0.0, transformer_->relative_pose_delta()->x, 1e-6);

  // 走到 0.5m 处退化 → 该位姿冻结为局部坐标原点
  publish_odom(0.5, 0.0, 0.0);
  std_msgs::msg::Bool flag;
  flag.data = true;
  pub_degen_->publish(flag);
  spin_for(150ms);
  ASSERT_TRUE(transformer_->relative_mode());
  EXPECT_NEAR(0.0, transformer_->relative_pose_delta()->x, 1e-6);

  // 退化期间再前进 0.3m → 局部差 = 相对原点的位移
  publish_odom(0.8, 0.0, 0.0);
  const auto delta = transformer_->relative_pose_delta();
  ASSERT_TRUE(delta.has_value());
  EXPECT_NEAR(0.3, delta->x, 1e-6);
  EXPECT_NEAR(0.0, delta->y, 1e-6);
}

// ── 4. TF 失败路径 ─────────────────────────────────────────────────────────

TEST_F(CoordinateTransformerTest, 无TF时变换失败并标记不可靠)
{
  geometry_msgs::msg::PointStamped in;
  in.header.frame_id = "livox_frame";
  in.point.x = 1.0;
  geometry_msgs::msg::PointStamped out;

  EXPECT_FALSE(transformer_->transform_to_world(in, out));
  EXPECT_TRUE(transformer_->tf_unreliable());
  EXPECT_FALSE(transformer_->last_error().empty());
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
