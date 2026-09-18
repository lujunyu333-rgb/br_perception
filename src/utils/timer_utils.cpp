#include "br_perception/utils/timer_utils.hpp"

#include <algorithm>
#include <cmath>

namespace br_perception {
namespace utils {

// ═══════════════════════════════════════════════════════════════════════════
// 1. Timer / ScopedTimer (§7.3 第 1 条)
// ═══════════════════════════════════════════════════════════════════════════

void Timer::start() noexcept
{
  start_time_ = SteadyClock::now();
  running_ = true;
}

double Timer::stop() noexcept
{
  if (!running_) {
    return last_ms_;        // 从未 start / 已 stop → 幂等返回上次结果 (初值 0.0)
  }
  const SteadyClock::time_point now = SteadyClock::now();
  last_ms_ = std::chrono::duration<double, std::milli>(now - start_time_).count();
  running_ = false;
  return last_ms_;
}

double Timer::elapsed_ms() const noexcept
{
  if (!running_) {
    return last_ms_;
  }
  const SteadyClock::time_point now = SteadyClock::now();
  return std::chrono::duration<double, std::milli>(now - start_time_).count();
}

void Timer::reset() noexcept
{
  start_time_ = SteadyClock::time_point{};
  running_ = false;
  last_ms_ = 0.0;
}

ScopedTimer::ScopedTimer(double& out_ms) noexcept
  : out_ms_(out_ms)
{
  timer_.start();
}

ScopedTimer::~ScopedTimer() noexcept
{
  if (!stopped_) {
    out_ms_ = timer_.stop();
  }
}

double ScopedTimer::stop() noexcept
{
  // Timer::stop() 在已停止时返回上次结果 → 本函数天然可重复调用
  const double ms = timer_.stop();
  if (!stopped_) {
    stopped_ = true;
    out_ms_ = ms;
  }
  return ms;
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. StageProfiler (§7.3 第 2 条)
// ═══════════════════════════════════════════════════════════════════════════

std::size_t StageProfiler::declare(const std::string& name)
{
  if (count_ >= kMaxStages) {
    return kInvalidStage;
  }
  // 可能抛 bad_alloc —— 本函数只允许在初始化期调用 (见头文件代价契约第 2 条)
  stages_[count_].name = name;
  stages_[count_].stats = LatencyStats{};

  const std::size_t idx = count_;
  ++count_;
  return idx;
}

const std::string& StageProfiler::name(std::size_t idx) const noexcept
{
  // 函数内静态对象: C++11 起初始化线程安全, 且空串走 SSO 不分配
  static const std::string kEmpty;
  if (!valid_stage(idx)) {
    return kEmpty;
  }
  return stages_[idx].name;
}

void StageProfiler::record(std::size_t idx, double ms) noexcept
{
  if (!valid_stage(idx)) {
    return;                          // 脏数据: 下标非法
  }
  if (!std::isfinite(ms) || ms < 0.0) {
    return;                          // 脏数据: NaN / ±Inf / 负数
  }

  LatencyStats& s = stages_[idx].stats;
  if (s.count == 0) {
    // 首个样本直接落位 —— 若走下面的增量式公式会出现 0/0
    s.mean_ms = ms;
    s.min_ms  = ms;
    s.max_ms  = ms;
  } else {
    // 增量式算术平均: mean_new = mean_old + (x - mean_old) / n_new
    s.mean_ms += (ms - s.mean_ms) / static_cast<double>(s.count + 1);
    s.min_ms   = std::min(s.min_ms, ms);
    s.max_ms   = std::max(s.max_ms, ms);
  }
  s.last_ms = ms;
  ++s.count;                         // 只有有效样本才计数 → count 长期为 0 是可观测信号

  frame_ms_[idx] = static_cast<float>(ms);
}

void StageProfiler::begin_frame() noexcept
{
  frame_ms_.fill(0.0f);
}

LatencyStats StageProfiler::stats(std::size_t idx) const noexcept
{
  if (!valid_stage(idx)) {
    return LatencyStats{};           // count == 0, 调用方可据此判定"无数据"
  }
  return stages_[idx].stats;
}

void StageProfiler::reset_stats() noexcept
{
  for (std::size_t i = 0; i < count_; ++i) {
    stages_[i].stats = LatencyStats{};
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. FpsCounter (§7.3 第 3 条)
// ═══════════════════════════════════════════════════════════════════════════

FpsCounter::FpsCounter(std::size_t window)
  : window_(window == 0 ? 1 : window)
{
}

void FpsCounter::tick()
{
  tick(SteadyClock::now());
}

void FpsCounter::tick(const SteadyClock::time_point& now)
{
  if (last_.has_value()) {
    const double dt_ms =
        std::chrono::duration<double, std::milli>(now - *last_).count();

    // 丢弃非正间隔 (同一时刻重复 tick / 假时钟回拨): 计入会让 fps() 变成 inf 或负值
    if (dt_ms > 0.0) {
      intervals_ms_.push_back(dt_ms);
      interval_sum_ms_ += dt_ms;
      last_interval_ms_ = dt_ms;

      // 滑动窗口: 挤出最老的样本, 并**同步**从累加和里减掉
      // (fps() 直接读 interval_sum_ms_, 这里漏减会静默漂移)
      while (intervals_ms_.size() > window_) {
        interval_sum_ms_ -= intervals_ms_.front();
        intervals_ms_.pop_front();
      }
    }
  }
  last_ = now;                       // 时间戳始终前移, 即使本次间隔被丢弃
}

double FpsCounter::fps() const noexcept
{
  if (intervals_ms_.empty() || interval_sum_ms_ <= 0.0) {
    return 0.0;
  }
  // 等价于 window 内样本数 / 累计秒数
  const double mean_ms = interval_sum_ms_ / static_cast<double>(intervals_ms_.size());
  return mean_ms > 0.0 ? (1000.0 / mean_ms) : 0.0;
}

double FpsCounter::last_interval_ms() const noexcept
{
  return last_interval_ms_;
}

void FpsCounter::reset() noexcept
{
  intervals_ms_.clear();
  interval_sum_ms_ = 0.0;
  last_interval_ms_ = 0.0;
  last_.reset();
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. 端到端延迟告警判定 (§7.3 第 5 条)
// ═══════════════════════════════════════════════════════════════════════════

LatencyLevel classify_latency(double end_to_end_ms,
                              const LatencyThresholds& thresholds)
{
  // 非有限值 (NaN / ±Inf) → ERROR: 异常输入不能掉进 OK
  if (!std::isfinite(end_to_end_ms)) {
    return LatencyLevel::ERROR;
  }

  const double warn_ms  = thresholds.warn_ms;
  const double error_ms = thresholds.error_ms;

  if (warn_ms > error_ms) {
    // 阈值非法 (例如 yaml 把两条配反了) → 退化为单一门限, 不做静默夹取
    return end_to_end_ms > error_ms ? LatencyLevel::ERROR : LatencyLevel::OK;
  }
  if (end_to_end_ms > error_ms) {
    return LatencyLevel::ERROR;
  }
  if (end_to_end_ms > warn_ms) {
    return LatencyLevel::WARN;
  }
  return LatencyLevel::OK;
}

const char* to_string(LatencyLevel level) noexcept
{
  switch (level) {
    case LatencyLevel::OK:    return "OK";
    case LatencyLevel::WARN:  return "WARN";
    case LatencyLevel::ERROR: return "ERROR";
  }
  return "UNKNOWN";                  // 枚举被扩展时的兜底
}

}  // namespace utils
}  // namespace br_perception
