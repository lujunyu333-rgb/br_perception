#pragma once

/// ═══════════════════════════════════════════════════════════════════════════════
/// mustika_detector.hpp — ROBOCON 2027 穆斯蒂卡相机检测节点
///
/// 依赖: OpenCV (core + imgproc) + cv_bridge + color_utils
/// 职责:
///   1. 订阅双路预处理图像 (/perception/camera_front/preprocessed,
///      /perception/camera_rear/preprocessed)
///   2. HSV 金色区域提取 (阈值从 color_thresholds.yaml 加载)
///   3. 形态学闭运算填补球体表面暗斑
///   4. cv::HoughCircles 检测圆形目标
///   5. 候选验证: 半径范围检查 + 金色像素占比 > 60% + 圆形度 > 0.85
///   6. Fallback: 金色区域轮廓检测 + minEnclosingCircle
///   7. 发布 /perception/camera/mustika (CameraMustika)
///   8. 发布 /perception/camera/mustika_diagnostics
///
/// 策略: 纯传统 CV — 金色球体在 HSV 空间极其明显, 不需要深度学习.
/// ═══════════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "br_perception/msg/camera_mustika.hpp"          // 自定义消息 (ROBOCON 专用)

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <string>
#include <vector>
#include <mutex>

namespace br_perception {
namespace camera {

/**
 * @brief 单次 Mustika 图像检测的原始结果
 *
 * 在 publish 之前聚合前后相机结果。
 */
struct MustikaImageResult
{
  bool   detected = false;
  float  u = 0.0f;
  float  v = 0.0f;
  float  radius = 0.0f;
  float  confidence = 0.0f;

  // 验证指标（诊断用）
  float  golden_ratio = 0.0f;       // 圆内金色像素占比
  float  circularity = 0.0f;        // 圆形度
  int    method = 0;                // 0=none, 1=HoughCircles, 2=contour fallback
};

/**
 * @brief 穆斯蒂卡相机检测组件
 *
 * 检测管线 (每路相机独立):
 *   RGB 图像 → BGR → HSV → 金色阈值 mask
 *   → 形态学闭运算 → HoughCircles
 *   → 候选验证 (半径 / 金色占比 / 圆形度)
 *   → [fallback] 轮廓 + minEnclosingCircle
 *   → 发布 CameraMustika + Diagnostics
 *
 * 前后相机独立回调, 共享参数和 YAML 颜色阈值。
 */
class MustikaDetector : public rclcpp::Node
{
public:
  explicit MustikaDetector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ── ROS 回调 ──
  void front_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void rear_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void process_and_publish(
      const sensor_msgs::msg::Image::SharedPtr msg,
      uint8_t camera_id,
      const rclcpp::Publisher<br_perception::msg::CameraMustika>::SharedPtr& pub);

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter>& params);

  // ═════════════════════════════════════════════════════════════════════════════
  // 检测管线子步骤
  // ═════════════════════════════════════════════════════════════════════════════

  /// @brief 完整检测管线: BGR 图像 → MustikaImageResult
  /// @param bgr         输入 BGR 图像 (CV_8UC3)
  /// @param cam_name    相机名 (front/rear, 日志用)
  /// @return 检测结果
  MustikaImageResult detect(const cv::Mat& bgr,
                            const std::string& cam_name);

  /// @brief Step 1: 生成金色区域二值 mask (HSV 阈值 + 形态学闭运算)
  /// @param bgr  输入 BGR
  /// @param mask 输出二值 mask (CV_8UC1, 255=金色)
  void extract_golden_mask(const cv::Mat& bgr, cv::Mat& mask);

  /// @brief Step 2: HoughCircles 检测圆形
  /// @param bgr         BGR 图像 (用于 HoughCircles 梯度)
  /// @param mask        金色区域 mask (用于预筛选 ROI)
  /// @param circles     输出圆列表 [x, y, r] × N (CV_32FC3 或空)
  void detect_circles_hough(const cv::Mat& bgr,
                            const cv::Mat& mask,
                            std::vector<cv::Vec3f>& circles);

  /// @brief Step 3: 验证检测到的圆
  /// @param bgr     BGR 图像
  /// @param mask    金色 mask
  /// @param circle  候选圆 (x, y, r)
  /// @param result  输出验证后的结果 (填充 golden_ratio, circularity, confidence)
  /// @return true 若通过验证
  bool validate_circle(const cv::Mat& bgr,
                       const cv::Mat& mask,
                       const cv::Vec3f& circle,
                       MustikaImageResult& result);

  /// @brief Step 4 (Fallback): 金色轮廓 + minEnclosingCircle
  /// @param mask    金色 mask
  /// @param result  输出结果
  /// @return true 若找到有效轮廓
  bool fallback_contour_detect(const cv::Mat& mask,
                               MustikaImageResult& result);

  /// @brief 计算轮廓的圆形度
  /// @param contour  输入轮廓点集
  /// @param area     轮廓面积 (传入避免重复计算)
  /// @return circularity = 4π × area / perimeter²  [0.0, 1.0]
  static float compute_circularity(const std::vector<cv::Point>& contour,
                                   double area);

  // ── 诊断 ──
  void publish_diagnostics(uint8_t camera_id,
                           double total_ms,
                           const MustikaImageResult& result,
                           int image_w, int image_h);

  // ═════════════════════════════════════════════════════════════════════════════
  // 订阅 & 发布
  // ═════════════════════════════════════════════════════════════════════════════
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_front_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rear_;

  rclcpp::Publisher<br_perception::msg::CameraMustika>::SharedPtr pub_front_;
  rclcpp::Publisher<br_perception::msg::CameraMustika>::SharedPtr pub_rear_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;

  // ── 参数回调句柄 ──
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ═════════════════════════════════════════════════════════════════════════════
  // HSV 金色阈值 (从 color_thresholds.yaml 加载)
  // ═════════════════════════════════════════════════════════════════════════════
  struct HsvGroup {
    cv::Scalar lower;
    cv::Scalar upper;
  };
  std::vector<HsvGroup> golden_groups_;  // 金色阈值组 (并集)
  std::string color_yaml_path_;

  // ═════════════════════════════════════════════════════════════════════════════
  // 检测参数 (可通过 ROS param 动态调整)
  // ═════════════════════════════════════════════════════════════════════════════
  // ── 形态学 ──
  int    morph_close_kernel_{5};       // 闭运算核大小
  int    morph_close_iters_{2};        // 闭运算迭代次数

  // ── HoughCircles ──
  double hough_dp_{1.0};              // 累加器分辨率反比
  double hough_min_dist_{30.0};       // 圆心最小间距 (pixels)
  double hough_param1_{100.0};        // Canny 高阈值
  double hough_param2_{30.0};         // 圆心累加器阈值
  int    hough_min_radius_{10};       // 最小半径 (pixels)
  int    hough_max_radius_{150};      // 最大半径 (pixels)

  // ── 验证阈值 ──
  double min_golden_ratio_{0.60};     // 圆内金色像素占比下限
  double min_circularity_{0.85};      // 圆形度下限

  // ── Fallback ──
  int    contour_min_area_{50};       // 轮廓最小面积 (pixels²)
  double contour_min_golden_ratio_{0.50}; // fallback 金色占比放宽

  // ── 相机参数 (用于半径合理性换算) ──
  double focal_length_px_{600.0};     // 近似焦距 (pixels)
  double mustika_real_radius_{0.065}; // 穆斯蒂卡真实半径 (m)
  double min_distance_{0.2};          // 最近检测距离 (m)
  double max_distance_{3.0};          // 最远检测距离 (m)

  // ═════════════════════════════════════════════════════════════════════════════
  // 运行时状态
  // ═════════════════════════════════════════════════════════════════════════════
  bool front_in_view_{false};
  bool rear_in_view_{false};
  std::mutex state_mutex_;

  // 帧计数器
  uint64_t front_frame_seq_{0};
  uint64_t rear_frame_seq_{0};
  std::mutex diag_mutex_;
};

}  // namespace camera
}  // namespace br_perception
