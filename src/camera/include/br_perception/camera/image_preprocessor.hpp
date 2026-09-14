#pragma once

/// ═══════════════════════════════════════════════════════════════════════════════
/// image_preprocessor.hpp — ROBOCON 2027 双相机图像预处理节点
///
/// 依赖: OpenCV (core + imgproc) + cv_bridge
/// 职责:
///   1. 订阅 /camera_front/image_raw 和 /camera_rear/image_raw
///   2. ROI 裁剪 (过滤无关注视区域)
///   3. CLAHE 自适应直方图均衡化 (LAB 色彩空间 L 通道)
///   4. 曝光补偿 (基于直方图拉伸的软件补偿)
///   5. Resize 到模型输入尺寸 (640×384)
///   6. BGR → RGB 转换
///   7. 可选归一化 (mean/std, 适配 ONNX 推理)
///   8. 发布预处理图像 + 诊断信息
///
/// 输出话题:
///   /perception/camera_front/preprocessed   (sensor_msgs::Image)
///   /perception/camera_rear/preprocessed    (sensor_msgs::Image)
///   /perception/camera/diagnostics          (diagnostic_msgs::DiagnosticArray)
/// ═══════════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <string>
#include <vector>
#include <mutex>

namespace br_perception {
namespace camera {

/**
 * @brief 双相机图像预处理组件
 *
 * 管线顺序 (每路独立并行):
 *   原始 BGR 图像 → ROI 裁剪 → CLAHE 增强 → 曝光补偿 → Resize → BGR→RGB → [归一化] → 发布
 *
 * 前后相机独立处理, 各自拥有独立的参数和回调,
 * 共享同一个节点和诊断发布器。
 */
class ImagePreprocessor : public rclcpp::Node
{
public:
  explicit ImagePreprocessor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ── ROS 回调 ──
  void front_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void rear_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter>& params);

  // ═════════════════════════════════════════════════════════════════════════════
  // 通用预处理管线 (前后相机共用)
  // ═════════════════════════════════════════════════════════════════════════════

  /// @brief 完整预处理管线
  /// @param bgr      输入 BGR 图像 (CV_8UC3)
  /// @param roi      ROI 裁剪区域 [x, y, w, h], 空或全零=不裁剪
  /// @param cla      CLAHE 参数 {enabled, clip_limit, tile_size}
  /// @param model    {target_w, target_h}
  /// @param norm     归一化参数 {enabled, mean[3], std[3]}
  /// @param out_enc  输出编码 (由管线判定并回填): "rgb8" 或 "32FC3"
  /// @return 预处理后的图像, 空 Mat 表示失败
  cv::Mat preprocess(const cv::Mat& bgr,
                     const std::vector<int64_t>& roi,
                     bool clahe_enabled, double clahe_clip, int clahe_tile,
                     int target_w, int target_h,
                     bool normalize_enabled,
                     const std::vector<double>& norm_mean,
                     const std::vector<double>& norm_std,
                     std::string& out_encoding);

  // ── 子步骤 ──

  /// @brief ROI 裁剪
  static cv::Mat crop_roi(const cv::Mat& image, const std::vector<int64_t>& roi);

  /// @brief CLAHE 增强 (LAB 色彩空间 L 通道, 避免颜色失真)
  static cv::Mat apply_clahe(const cv::Mat& bgr, double clip_limit, int tile_size);

  /// @brief 曝光补偿 — V 通道直方图拉伸 (clip 两端 0.5%)
  static cv::Mat compensate_exposure(const cv::Mat& bgr);

  /// @brief Resize 到模型输入尺寸
  static cv::Mat resize_to_model(const cv::Mat& image, int target_w, int target_h);

  /// @brief BGR → RGB
  static cv::Mat convert_bgr_to_rgb(const cv::Mat& bgr);

  /// @brief 归一化: (pixel/255.0 - mean) / std → float32
  static cv::Mat apply_normalization(const cv::Mat& rgb,
                                     const std::vector<double>& mean,
                                     const std::vector<double>& stddev);

  /// @brief 单路相机通用的回调处理逻辑
  void process_and_publish(
      const sensor_msgs::msg::Image::SharedPtr msg,
      const std::string& cam_name,
      bool clahe_enabled, double clahe_clip, int clahe_tile,
      const std::vector<int64_t>& roi,
      const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub);

  // ── 诊断 ──
  void publish_diagnostics(const std::string& cam_name,
                           double preprocess_ms, size_t frame_seq,
                           int orig_w, int orig_h,
                           int out_w, int out_h,
                           const std::string& out_encoding);

  // ── 订阅 ──
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_front_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rear_;

  // ── 发布 ──
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_front_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rear_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;

  // ── 参数回调句柄 ──
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ═════════════════════════════════════════════════════════════════════════════
  // 全局模型参数
  // ═════════════════════════════════════════════════════════════════════════════
  int    model_input_w_{640};
  int    model_input_h_{384};
  bool   enable_normalize_{false};
  std::vector<double> norm_mean_{0.485, 0.456, 0.406};
  std::vector<double> norm_std_{0.229, 0.224, 0.225};

  // ═════════════════════════════════════════════════════════════════════════════
  // 前向相机参数
  // ═════════════════════════════════════════════════════════════════════════════
  bool   front_clahe_enabled_{true};
  double front_clahe_clip_{2.0};
  int    front_clahe_tile_{8};
  std::vector<int64_t> front_roi_{0, 0, 1280, 720};
  int    front_orig_w_{1280};
  int    front_orig_h_{720};

  // ═════════════════════════════════════════════════════════════════════════════
  // 后向相机参数
  // ═════════════════════════════════════════════════════════════════════════════
  bool   rear_clahe_enabled_{true};
  double rear_clahe_clip_{2.0};
  int    rear_clahe_tile_{8};
  std::vector<int64_t> rear_roi_{0, 0, 1280, 720};
  int    rear_orig_w_{1280};
  int    rear_orig_h_{720};

  // ── 帧计数器 (诊断用) ──
  uint64_t front_frame_seq_{0};
  uint64_t rear_frame_seq_{0};

  // ── 互斥锁 (前/后相机回调可并发) ──
  std::mutex diag_mutex_;
};

}  // namespace camera
}  // namespace br_perception
