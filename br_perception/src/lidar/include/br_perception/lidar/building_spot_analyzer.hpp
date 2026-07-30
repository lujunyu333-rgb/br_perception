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

// ── 建筑位状态 ──
enum class BuildingSpotStatus : uint8_t {
  EMPTY          = 0,   // 空位 — 无方块或高度可忽略
  ONE_EARTH      = 1,   // 一块地球方块 (≈350mm)
  TWO_EARTH      = 2,   // 两块地球方块 (≈700mm)
  COMPLETE_TOWER = 3,   // 完整塔: 两块地球 + 一块天空 (≈900mm)
};

// ── 单个建筑位检测结果 ──
struct BuildingSpot {
  int   id = 0;
  float spot_x = 0;            // 建筑位中心 X (场地坐标系)
  float spot_y = 0;            // 建筑位中心 Y
  float platform_z = 0;        // 平台表面高度

  BuildingSpotStatus status = BuildingSpotStatus::EMPTY;

  float highest_z = 0;         // 柱状区域内最高点 Z
  float height = 0;            // 最高点到平台表面的高度差

  int   point_count = 0;       // 区域内非地面点数

  bool  has_top_surface = false;   // 是否检测到近水平顶面
  float top_surface_z = 0;         // 顶面平均高度 (如有)
  int   top_surface_points = 0;    // 顶面点数

  bool  needs_visual_check = false; // 完整塔需要视觉确认天空方块颜色
};

/**
 * @brief 建筑位分析节点 — ROBOCON 2027 场地建筑塔状态检测
 *
 * 管线:
 *   非地面点云 + field_geometry.yaml 建筑位坐标
 *   → 逐建筑位裁剪垂直柱状点云
 *   → 检测顶面 & 计算高度
 *   → 按高度阈值分类 (EMPTY / ONE_EARTH / TWO_EARTH / COMPLETE_TOWER)
 *
 * 输入:
 *   /perception/lidar/non_ground  — 非地面点云 (sensor_msgs::PointCloud2)
 *
 * 输出:
 *   /perception/lidar/building_spots  — 建筑位占用状态 (MarkerArray, 彩色方块)
 *   /perception/lidar/diagnostics      — 耗时 + 各状态计数 (DiagnosticArray)
 *
 * 注意:
 *   雷达无法判断天空方块的朝上颜色 → 完整塔标记 needs_visual_check = true
 */
class BuildingSpotAnalyzer : public rclcpp::Node
{
public:
  using PointCloud    = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = PointCloud::Ptr;

  explicit BuildingSpotAnalyzer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ── ROS 回调 ──
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_param_change(
      const std::vector<rclcpp::Parameter>& params);

  // ── 核心管线 ──
  /// 分析单个建筑位: 裁剪柱状区域 → 点数检查 → 顶面检测 → 高度分类
  BuildingSpot analyze_spot(const PointCloudPtr& cloud, int id,
                            float spot_x, float spot_y, float platform_z);

  /// 检测点云中是否存在近水平顶面 (最高点附近的点聚类)
  /// @return {has_surface, surface_z, surface_points}
  std::tuple<bool, float, int> detect_top_surface(
      const PointCloudPtr& spot_cloud, float z_max);

  // ── 发布 ──
  void publish_spots(const std::vector<BuildingSpot>& spots,
                     const std_msgs::msg::Header& header);
  void publish_diagnostics(double total_ms,
                           const std::vector<BuildingSpot>& spots);

  // ── 话题 ──
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_spots_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ══════════════════════════════════════════════════════════════
  // 建筑位坐标参数 (从 field_geometry.yaml 通过 ROS params 传入)
  // ══════════════════════════════════════════════════════════════
  std::vector<std::pair<double, double>> building_positions_;

  // ══════════════════════════════════════════════════════════════
  // 几何参数
  // ══════════════════════════════════════════════════════════════
  double platform_z_{0.0};           // 平台表面绝对高度 (场地 Z 坐标, m)
  double spot_half_size_{0.25};      // 建筑位半宽 (500×500mm → 250mm 半宽)
  double column_z_min_{0.0};         // 柱状区域 Z 下界偏移 (相对于 platform_z)
  double column_z_max_{1.5};         // 柱状区域 Z 上界偏移

  // ══════════════════════════════════════════════════════════════
  // 检测阈值
  // ══════════════════════════════════════════════════════════════
  int    min_points_threshold_{5};   // 低于此值 → EMPTY
  double top_surface_tolerance_{0.03}; // 顶面点 Z 容差 (m)
  int    top_surface_min_points_{3};   // 顶面最小点数

  // ══════════════════════════════════════════════════════════════
  // 高度分类阈值 (相对于 platform_z, 单位 m)
  // ══════════════════════════════════════════════════════════════
  double empty_max_height_{0.05};      // 高度 < 此值 → EMPTY
  double one_earth_min_{0.20};         // ONE_EARTH 下界
  double one_earth_max_{0.45};         // ONE_EARTH 上界
  double two_earth_min_{0.55};         // TWO_EARTH 下界
  double two_earth_max_{0.80};         // TWO_EARTH 上界
  double complete_tower_min_{0.80};    // COMPLETE_TOWER 下界
  double complete_tower_max_{1.10};    // COMPLETE_TOWER 上界
};

}  // namespace lidar
}  // namespace br_perception
