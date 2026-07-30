/// ═══════════════════════════════════════════════════════════════════════════════
/// image_preprocessor.cpp — ROBOCON 2027 双相机图像预处理管线实现
///
/// 管线 (每路独立):
///   ROS Image → CV::Mat(BGR) → ROI 裁剪 → CLAHE 增强(LAB L通道)
///   → 曝光补偿(V通道直方图拉伸) → Resize(640×384) → BGR→RGB
///   → [可选归一化 mean/std] → ROS Image → publish
///
/// 关键设计决策:
///   - CLAHE 应用于 LAB-L 通道 (避免 RGB 三通道独立均衡导致偏色)
///   - 曝光补偿用 V 通道直方图 clip 两端 0.5% → min-max 拉伸
///   - 前后相机回调独立, 共享 diag_mutex_ 保护诊断发布
///   - cv_bridge 只在回调入口/出口各调用一次 (零拷贝 cv_bridge::toCvShare)
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/image_preprocessor.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <string>
#include <algorithm>
#include <stdexcept>

namespace br_perception {
namespace camera {

// ═══════════════════════════════════════════════════════════════════════════════
// 构造 & 参数初始化
// ═══════════════════════════════════════════════════════════════════════════════

ImagePreprocessor::ImagePreprocessor(const rclcpp::NodeOptions& options)
  : Node("image_preprocessor", options)
{
  // ── 全局模型参数 ──
  this->declare_parameter<int>("model_input_w", model_input_w_);
  this->declare_parameter<int>("model_input_h", model_input_h_);
  this->declare_parameter<bool>("enable_normalize", enable_normalize_);
  this->declare_parameter<std::vector<double>>("norm_mean", norm_mean_);
  this->declare_parameter<std::vector<double>>("norm_std", norm_std_);

  // ── 前向相机参数 ──
  this->declare_parameter<bool>("front_clahe_enabled", front_clahe_enabled_);
  this->declare_parameter<double>("front_clahe_clip", front_clahe_clip_);
  this->declare_parameter<int>("front_clahe_tile", front_clahe_tile_);
  this->declare_parameter<std::vector<int64_t>>("front_roi", front_roi_);

  // ── 后向相机参数 ──
  this->declare_parameter<bool>("rear_clahe_enabled", rear_clahe_enabled_);
  this->declare_parameter<double>("rear_clahe_clip", rear_clahe_clip_);
  this->declare_parameter<int>("rear_clahe_tile", rear_clahe_tile_);
  this->declare_parameter<std::vector<int64_t>>("rear_roi", rear_roi_);

  // ── 读取所有参数 ──
  this->get_parameter("model_input_w", model_input_w_);
  this->get_parameter("model_input_h", model_input_h_);
  this->get_parameter("enable_normalize", enable_normalize_);
  this->get_parameter("norm_mean", norm_mean_);
  this->get_parameter("norm_std", norm_std_);

  this->get_parameter("front_clahe_enabled", front_clahe_enabled_);
  this->get_parameter("front_clahe_clip", front_clahe_clip_);
  this->get_parameter("front_clahe_tile", front_clahe_tile_);
  this->get_parameter("front_roi", front_roi_);

  this->get_parameter("rear_clahe_enabled", rear_clahe_enabled_);
  this->get_parameter("rear_clahe_clip", rear_clahe_clip_);
  this->get_parameter("rear_clahe_tile", rear_clahe_tile_);
  this->get_parameter("rear_roi", rear_roi_);

  // ── 规范化参数 ──
  model_input_w_ = std::max(16, std::min(model_input_w_, 4096));
  model_input_h_ = std::max(16, std::min(model_input_h_, 4096));
  front_clahe_tile_ = std::max(2, std::min(front_clahe_tile_, 64));
  rear_clahe_tile_  = std::max(2, std::min(rear_clahe_tile_,  64));
  front_clahe_clip_ = std::max(0.0, std::min(front_clahe_clip_, 40.0));
  rear_clahe_clip_  = std::max(0.0, std::min(rear_clahe_clip_,  40.0));

  if (norm_mean_.size() != 3) norm_mean_ = {0.485, 0.456, 0.406};
  if (norm_std_.size()  != 3) norm_std_  = {0.229, 0.224, 0.225};

  // ── 参数动态更新 ──
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ImagePreprocessor::on_parameter_change, this, std::placeholders::_1));

  // ── 订阅原始图像 ──
  sub_front_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera_front/image_raw", rclcpp::SystemDefaultsQoS(),
    std::bind(&ImagePreprocessor::front_callback, this, std::placeholders::_1));

  sub_rear_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera_rear/image_raw", rclcpp::SystemDefaultsQoS(),
    std::bind(&ImagePreprocessor::rear_callback, this, std::placeholders::_1));

  // ── 发布预处理图像 ──
  pub_front_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/perception/camera_front/preprocessed", rclcpp::SystemDefaultsQoS());

  pub_rear_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/perception/camera_rear/preprocessed", rclcpp::SystemDefaultsQoS());

  // ── 诊断 ──
  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/perception/camera/diagnostics", 10);

  RCLCPP_INFO(this->get_logger(),
    "ImagePreprocessor ready | model=%dx%d | "
    "front(clahe=%s clip=%.1f tile=%d roi=%ld×%ld+%ld+%ld) | "
    "rear(clahe=%s clip=%.1f tile=%d roi=%ld×%ld+%ld+%ld) | "
    "normalize=%s (mean=[%.3f,%.3f,%.3f] std=[%.3f,%.3f,%.3f])",
    model_input_w_, model_input_h_,
    front_clahe_enabled_ ? "on" : "off", front_clahe_clip_, front_clahe_tile_,
    front_roi_.size() >= 4 ? front_roi_[2] : 0, front_roi_.size() >= 4 ? front_roi_[3] : 0,
    front_roi_.size() >= 2 ? front_roi_[0] : 0, front_roi_.size() >= 2 ? front_roi_[1] : 0,
    rear_clahe_enabled_  ? "on" : "off", rear_clahe_clip_,  rear_clahe_tile_,
    rear_roi_.size()  >= 4 ? rear_roi_[2]  : 0, rear_roi_.size()  >= 4 ? rear_roi_[3]  : 0,
    rear_roi_.size()  >= 2 ? rear_roi_[0]  : 0, rear_roi_.size()  >= 2 ? rear_roi_[1]  : 0,
    enable_normalize_ ? "on" : "off",
    norm_mean_[0], norm_mean_[1], norm_mean_[2],
    norm_std_[0],  norm_std_[1],  norm_std_[2]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 前向相机回调
// ═══════════════════════════════════════════════════════════════════════════════
void ImagePreprocessor::front_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  process_and_publish(msg, "front",
                      front_clahe_enabled_, front_clahe_clip_, front_clahe_tile_,
                      front_roi_, pub_front_);
  front_frame_seq_++;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 后向相机回调
// ═══════════════════════════════════════════════════════════════════════════════
void ImagePreprocessor::rear_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  process_and_publish(msg, "rear",
                      rear_clahe_enabled_, rear_clahe_clip_, rear_clahe_tile_,
                      rear_roi_, pub_rear_);
  rear_frame_seq_++;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 单路相机通用处理
// ═══════════════════════════════════════════════════════════════════════════════
void ImagePreprocessor::process_and_publish(
    const sensor_msgs::msg::Image::SharedPtr msg,
    const std::string& cam_name,
    bool clahe_enabled, double clahe_clip, int clahe_tile,
    const std::vector<int64_t>& roi,
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub)
{
  // ── 计时起点 ──
  auto t0 = std::chrono::steady_clock::now();

  // ── 1. ROS Image → cv::Mat (BGR8) ──
  cv::Mat bgr;
  try {
    // 使用 toCvShare 零拷贝 (shared_ptr 保证数据生命周期)
    auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    bgr = cv_ptr->image;
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "[%s] cv_bridge error: %s", cam_name.c_str(), e.what());
    return;
  }

  if (bgr.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "[%s] Empty image received", cam_name.c_str());
    return;
  }

  const int orig_w = bgr.cols;
  const int orig_h = bgr.rows;

  // ── 2. 预处理管线 ──
  std::string out_encoding;
  cv::Mat processed = preprocess(bgr, roi,
                                 clahe_enabled, clahe_clip, clahe_tile,
                                 model_input_w_, model_input_h_,
                                 enable_normalize_, norm_mean_, norm_std_,
                                 out_encoding);

  if (processed.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "[%s] Preprocessing returned empty image", cam_name.c_str());
    return;
  }

  // ── 3. cv::Mat → ROS Image + 发布 ──
  auto out_msg = cv_bridge::CvImage(msg->header, out_encoding, processed).toImageMsg();
  pub->publish(*out_msg);

  // ── 4. 耗时统计 ──
  auto t1 = std::chrono::steady_clock::now();
  double total_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

  // ── 5. 诊断 ──
  publish_diagnostics(cam_name, total_ms,
                      (cam_name == "front") ? front_frame_seq_ : rear_frame_seq_,
                      orig_w, orig_h,
                      processed.cols, processed.rows,
                      out_encoding);

  // ── 延迟超标告警 ──
  if (total_ms > 20.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[%s] Preprocessing slow: %.1f ms (%dx%d → %dx%d)",
      cam_name.c_str(), total_ms, orig_w, orig_h,
      processed.cols, processed.rows);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 完整预处理管线
// ═══════════════════════════════════════════════════════════════════════════════
cv::Mat ImagePreprocessor::preprocess(
    const cv::Mat& bgr,
    const std::vector<int64_t>& roi,
    bool clahe_enabled, double clahe_clip, int clahe_tile,
    int target_w, int target_h,
    bool normalize_enabled,
    const std::vector<double>& norm_mean,
    const std::vector<double>& norm_std,
    std::string& out_encoding)
{
  // ── Step 1: ROI 裁剪 ──
  cv::Mat working = crop_roi(bgr, roi);
  if (working.empty()) return {};

  // ── Step 2: CLAHE 增强 (LAB L 通道) ──
  if (clahe_enabled) {
    working = apply_clahe(working, clahe_clip, clahe_tile);
  }

  // ── Step 3: 曝光补偿 ──
  working = compensate_exposure(working);

  // ── Step 4: Resize 到模型输入尺寸 ──
  working = resize_to_model(working, target_w, target_h);

  // ── Step 5: BGR → RGB ──
  working = convert_bgr_to_rgb(working);

  // ── Step 6: [可选] 归一化 ──
  if (normalize_enabled && norm_mean.size() >= 3 && norm_std.size() >= 3) {
    working = apply_normalization(working, norm_mean, norm_std);
    out_encoding = sensor_msgs::image_encodings::TYPE_32FC3;
  } else {
    // 确保输出为连续内存 (cv_bridge 发布要求)
    if (!working.isContinuous()) {
      working = working.clone();
    }
    out_encoding = sensor_msgs::image_encodings::RGB8;
  }

  return working;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 1: ROI 裁剪
// ═══════════════════════════════════════════════════════════════════════════════
cv::Mat ImagePreprocessor::crop_roi(const cv::Mat& image,
                                    const std::vector<int64_t>& roi)
{
  // ROI 无效或为全图 → 不裁剪
  if (roi.size() < 4) {
    return image;
  }

  const int x      = static_cast<int>(roi[0]);
  const int y      = static_cast<int>(roi[1]);
  const int w_raw  = static_cast<int>(roi[2]);
  const int h_raw  = static_cast<int>(roi[3]);

  // 全零 ROI 或覆盖全图 → 不裁剪
  if (x == 0 && y == 0 && w_raw >= image.cols && h_raw >= image.rows) {
    return image;
  }

  // 修正边界
  const int img_w = image.cols;
  const int img_h = image.rows;

  int cx = std::max(0, x);
  int cy = std::max(0, y);
  int cw = std::min(w_raw, img_w - cx);
  int ch = std::min(h_raw, img_h - cy);

  if (cw <= 0 || ch <= 0) {
    // ROI 完全在图像外, 返回原图
    return image;
  }

  cv::Rect rect(cx, cy, cw, ch);
  return image(rect).clone();  // clone necessary: ROI Mat data not contiguous
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 2: CLAHE 增强 — LAB 色彩空间 L 通道
// ═══════════════════════════════════════════════════════════════════════════════
cv::Mat ImagePreprocessor::apply_clahe(const cv::Mat& bgr,
                                       double clip_limit, int tile_size)
{
  // BGR → LAB
  cv::Mat lab;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);

  // 分离通道: L, a, b
  std::vector<cv::Mat> channels(3);
  cv::split(lab, channels);

  // 仅对 L 通道做 CLAHE (避免 a/b 通道产生颜色失真)
  auto clahe = cv::createCLAHE(clip_limit,
                                cv::Size(tile_size, tile_size));
  clahe->apply(channels[0], channels[0]);

  // 合并通道
  cv::merge(channels, lab);

  // LAB → BGR
  cv::Mat result;
  cv::cvtColor(lab, result, cv::COLOR_Lab2BGR);
  return result;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 3: 曝光补偿 — V 通道直方图拉伸
// ═══════════════════════════════════════════════════════════════════════════════
cv::Mat ImagePreprocessor::compensate_exposure(const cv::Mat& bgr)
{
  // BGR → HSV
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  std::vector<cv::Mat> channels(3);
  cv::split(hsv, channels);
  cv::Mat& v_channel = channels[2];  // V = value / brightness

  // ── 计算 V 通道直方图 ──
  int hist_size = 256;
  float range[] = {0, 256};
  const float* hist_range = {range};
  cv::Mat hist;
  cv::calcHist(&v_channel, 1, nullptr, cv::Mat(), hist, 1, &hist_size, &hist_range);

  // ── Clip 两端各 0.5% ──
  const double total = v_channel.total();
  const double clip_count = total * 0.005;  // 0.5%

  // 从低端累加
  int v_min = 0;
  {
    double accum = 0;
    for (int i = 0; i < 256; ++i) {
      accum += hist.at<float>(i);
      if (accum > clip_count) { v_min = i; break; }
    }
  }

  // 从高端累加
  int v_max = 255;
  {
    double accum = 0;
    for (int i = 255; i >= 0; --i) {
      accum += hist.at<float>(i);
      if (accum > clip_count) { v_max = i; break; }
    }
  }

  // ── Min-max 拉伸 ──
  if (v_max > v_min) {
    const double scale = 255.0 / static_cast<double>(v_max - v_min);
    for (int y = 0; y < v_channel.rows; ++y) {
      uint8_t* row = v_channel.ptr<uint8_t>(y);
      for (int x = 0; x < v_channel.cols; ++x) {
        int val = row[x];
        val = cv::saturate_cast<uint8_t>((val - v_min) * scale);
        row[x] = static_cast<uint8_t>(val);
      }
    }
  }

  // ── 合并回 BGR ──
  cv::merge(channels, hsv);
  cv::Mat result;
  cv::cvtColor(hsv, result, cv::COLOR_HSV2BGR);
  return result;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 4: Resize 到模型输入尺寸
// ═══════════════════════════════════════════════════════════════════════════════
cv::Mat ImagePreprocessor::resize_to_model(const cv::Mat& image,
                                           int target_w, int target_h)
{
  // 尺寸已匹配 → 不缩放
  if (image.cols == target_w && image.rows == target_h) {
    return image;
  }

  cv::Mat resized;
  // INTER_LINEAR: 速度与质量的平衡; 如需更高精度可换 INTER_CUBIC
  cv::resize(image, resized, cv::Size(target_w, target_h), 0, 0, cv::INTER_LINEAR);
  return resized;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 5: BGR → RGB
// ═══════════════════════════════════════════════════════════════════════════════
cv::Mat ImagePreprocessor::convert_bgr_to_rgb(const cv::Mat& bgr)
{
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return rgb;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 6: 归一化 — (pixel/255.0 - mean) / std → float32
// ═══════════════════════════════════════════════════════════════════════════════
cv::Mat ImagePreprocessor::apply_normalization(const cv::Mat& rgb,
                                               const std::vector<double>& mean,
                                               const std::vector<double>& stddev)
{
  CV_Assert(rgb.type() == CV_8UC3);
  CV_Assert(mean.size() >= 3 && stddev.size() >= 3);

  // 转为 float32 [0, 1]
  cv::Mat rgb_float;
  rgb.convertTo(rgb_float, CV_32FC3, 1.0 / 255.0);

  // (x - mean) / std → 逐通道
  // OpenCV 的 subtract + divide 对多通道自动按 Scalar 广播
  cv::Mat normalized;
  cv::subtract(rgb_float,
               cv::Scalar(static_cast<float>(mean[0]),
                          static_cast<float>(mean[1]),
                          static_cast<float>(mean[2])),
               normalized);
  cv::divide(normalized,
             cv::Scalar(static_cast<float>(stddev[0]),
                        static_cast<float>(stddev[1]),
                        static_cast<float>(stddev[2])),
             normalized);

  return normalized;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 诊断发布
// ═══════════════════════════════════════════════════════════════════════════════
void ImagePreprocessor::publish_diagnostics(
    const std::string& cam_name,
    double preprocess_ms,
    size_t frame_seq,
    int orig_w, int orig_h,
    int out_w, int out_h,
    const std::string& out_encoding)
{
  std::lock_guard<std::mutex> lock(diag_mutex_);

  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "image_preprocessor_" + cam_name;
  status.hardware_id = "camera_" + cam_name;

  // 总体健康判定
  if (preprocess_ms < 15.0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";
  } else if (preprocess_ms < 25.0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "SLOW";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "VERY SLOW";
  }

  // Key-value 对
  auto add_kv = [&](const std::string& key, double val) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = std::to_string(val);
    status.values.push_back(kv);
  };

  add_kv("preprocess_ms", preprocess_ms);
  add_kv("frame_seq",      static_cast<double>(frame_seq));
  add_kv("input_width",    static_cast<double>(orig_w));
  add_kv("input_height",   static_cast<double>(orig_h));
  add_kv("output_width",   static_cast<double>(out_w));
  add_kv("output_height",  static_cast<double>(out_h));

  diag_array.status.push_back(status);
  pub_diagnostics_->publish(diag_array);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 动态参数更新
// ═══════════════════════════════════════════════════════════════════════════════
rcl_interfaces::msg::SetParametersResult ImagePreprocessor::on_parameter_change(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto& param : params) {
    const std::string& name = param.get_name();

    try {
      // ── 全局模型参数 ──
      if (name == "model_input_w") {
        int val = param.as_int();
        if (val < 16 || val > 4096) {
          result.successful = false;
          result.reason = "model_input_w must be in [16, 4096]";
          break;
        }
        model_input_w_ = val;
      }
      else if (name == "model_input_h") {
        int val = param.as_int();
        if (val < 16 || val > 4096) {
          result.successful = false;
          result.reason = "model_input_h must be in [16, 4096]";
          break;
        }
        model_input_h_ = val;
      }
      else if (name == "enable_normalize") {
        enable_normalize_ = param.as_bool();
      }
      else if (name == "norm_mean") {
        auto val = param.as_double_array();
        if (val.size() != 3) {
          result.successful = false;
          result.reason = "norm_mean must have exactly 3 elements";
          break;
        }
        norm_mean_ = val;
      }
      else if (name == "norm_std") {
        auto val = param.as_double_array();
        if (val.size() != 3) {
          result.successful = false;
          result.reason = "norm_std must have exactly 3 elements";
          break;
        }
        // std 不能有零值, 否则除零
        for (size_t i = 0; i < val.size(); ++i) {
          if (std::abs(val[i]) < 1e-9) {
            result.successful = false;
            result.reason = "norm_std elements must be non-zero";
            break;
          }
        }
        if (!result.successful) break;
        norm_std_ = val;
      }

      // ── 前向相机参数 ──
      else if (name == "front_clahe_enabled") {
        front_clahe_enabled_ = param.as_bool();
      }
      else if (name == "front_clahe_clip") {
        double val = param.as_double();
        if (val < 0.0 || val > 40.0) {
          result.successful = false;
          result.reason = "front_clahe_clip must be in [0, 40]";
          break;
        }
        front_clahe_clip_ = val;
      }
      else if (name == "front_clahe_tile") {
        int val = param.as_int();
        if (val < 2 || val > 64) {
          result.successful = false;
          result.reason = "front_clahe_tile must be in [2, 64]";
          break;
        }
        front_clahe_tile_ = val;
      }
      else if (name == "front_roi") {
        auto val = param.as_integer_array();
        if (val.size() != 4) {
          result.successful = false;
          result.reason = "front_roi must have exactly 4 elements [x, y, w, h]";
          break;
        }
        front_roi_ = val;
      }

      // ── 后向相机参数 ──
      else if (name == "rear_clahe_enabled") {
        rear_clahe_enabled_ = param.as_bool();
      }
      else if (name == "rear_clahe_clip") {
        double val = param.as_double();
        if (val < 0.0 || val > 40.0) {
          result.successful = false;
          result.reason = "rear_clahe_clip must be in [0, 40]";
          break;
        }
        rear_clahe_clip_ = val;
      }
      else if (name == "rear_clahe_tile") {
        int val = param.as_int();
        if (val < 2 || val > 64) {
          result.successful = false;
          result.reason = "rear_clahe_tile must be in [2, 64]";
          break;
        }
        rear_clahe_tile_ = val;
      }
      else if (name == "rear_roi") {
        auto val = param.as_integer_array();
        if (val.size() != 4) {
          result.successful = false;
          result.reason = "rear_roi must have exactly 4 elements [x, y, w, h]";
          break;
        }
        rear_roi_ = val;
      }

      else {
        // 未知参数, 忽略 (可能属于其他节点)
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
      "Params updated | model=%dx%d | front_clahe=%s(%.1f,%d) rear_clahe=%s(%.1f,%d) | normalize=%s",
      model_input_w_, model_input_h_,
      front_clahe_enabled_ ? "on" : "off", front_clahe_clip_, front_clahe_tile_,
      rear_clahe_enabled_  ? "on" : "off", rear_clahe_clip_,  rear_clahe_tile_,
      enable_normalize_    ? "on" : "off");
  } else {
    RCLCPP_WARN(this->get_logger(), "Param update rejected: %s", result.reason.c_str());
  }

  return result;
}

}  // namespace camera
}  // namespace br_perception

// ── ROS2 组件注册 ──
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::camera::ImagePreprocessor)
