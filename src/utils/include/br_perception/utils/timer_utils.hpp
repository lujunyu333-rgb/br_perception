#pragma once

/// ═══════════════════════════════════════════════════════════════════════════
/// timer_utils.hpp — ROBOCON 2027 高精度计时与分模块耗时统计
///
/// 依赖: 无 (纯 C++17 标准库; 不含任何 ROS / PCL / OpenCV 头)
/// 职责 (任务书 §7.3):
///   1. 高精度计时器                              → Timer / ScopedTimer
///   2. 分模块耗时统计 (次数/最近/平均/最小/最大)   → StageProfiler + LatencyStats
///   3. 帧率计数器 (滑动窗口平均)                  → FpsCounter
///   4. 端到端延迟超标 (>80ms) 告警判定            → classify_latency
///
/// ⚠ 与任务书 §7.3 的两处**有意偏差** (评审时请重点看这两条):
///
///   a) §7.3 第 4 条「发布耗时统计为 ROS 话题 /perception/timing」**不在本文件**。
///      发布是薄壳节点的职责 —— 见 doc/容器化重构设计_20260914.md §4 硬性规则第 1 条
///      「算法类不碰 ROS: 构造不接 Node*、不 declare_parameter、无订阅发布」。
///      正因如此本文件可被 gtest 直接单测 (任务书 §11.1), 不需要起 ROS。
///      落点: 将来由 lidar_perception_node / perception_pipeline 的薄壳读取
///      StageProfiler 与 classify_latency 的结果后发布。
///
///   b) §7.3 写 high_resolution_clock, 本实现用 **steady_clock**。
///      在 libstdc++ (GCC / Ubuntu 22.04) 上 high_resolution_clock 是 system_clock 的
///      别名, **不单调** —— 受 NTP 校时会回跳, 测量间隔可能得到负值。
///      测量时间间隔必须用单调时钟, 故此处偏离任务书的字面写法。
///
/// 线程安全: **非线程安全**。设计上由容器内单线程顺序调用 (设计文档 §3.1)。
///           将来 health_monitor 若需跨线程读取统计, 由调用方加锁 ——
///           不在此处引入 mutex, 避免给热路径加锁开销。
///
/// 代价契约 (实现 .cpp 时必须遵守, 三条):
///   1. 热路径上的成员 —— record() / frame_ms() / fps() / elapsed_ms() ——
///      **不得分配堆内存、不得加锁**
///   2. 允许分配的只有初始化与低频操作: declare()、FpsCounter 的构造与 tick()
///      (deque 增长; tick() 稳定后不再分配, 但不做保证)
///   3. 收到脏数据 (下标非法 / 负数 / NaN / Inf) 一律**静默丢弃** ——
///      统计模块绝不能把感知管线搞崩
/// ═══════════════════════════════════════════════════════════════════════════

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>

namespace br_perception {
namespace utils {

/// 计时统一使用的单调时钟 (见文件头偏差 b)
using SteadyClock = std::chrono::steady_clock;

// ═══════════════════════════════════════════════════════════════════════════
// 1. 计时器 (§7.3 第 1 条)
// ═══════════════════════════════════════════════════════════════════════════

/// @brief 高精度计时器 (单调时钟), 可重复使用。
///
/// 典型用法:
/// @code
///   Timer t;
///   t.start();
///   do_work();
///   const double ms = t.stop();     // → LidarResult.stage_elapsed_ms
/// @endcode
class Timer
{
public:
  /// 开始计时。若上一轮未 stop, 直接丢弃 (不报错、不累加)
  void start() noexcept;

  /// @brief 停止计时并返回本次耗时 (ms)。
  /// 从未 start 过 → 返回最近一次结果 (初值 0.0)
  [[nodiscard]] double stop() noexcept;

  /// @brief 取当前耗时 (ms) 且**不**停止计时 — 用于中途打点。
  /// 未在计时中 → 返回最近一次 stop() 的结果
  [[nodiscard]] double elapsed_ms() const noexcept;

  /// 是否正在计时
  [[nodiscard]] bool running() const noexcept { return running_; }

  /// 清除计时状态与最近一次结果 (回到刚构造的状态)
  void reset() noexcept;

private:
  SteadyClock::time_point start_time_{};
  bool running_{false};
  double last_ms_{0.0};
};

/// @brief RAII 作用域计时: 构造即开始, 析构把耗时 (ms) 写入外部变量。
///
/// 用于「进作用域就开始、出作用域就记一笔」, 避免手工 start/stop 漏写:
/// @code
///   double ms = 0.0;
///   {
///     ScopedTimer st(ms);
///     preprocess(cloud);
///   }                               // 出作用域, ms 已是本次耗时
///   profiler.record(kPreprocess, ms);
/// @endcode
///
/// ⚠ out_ms 引用的对象必须在 ScopedTimer 存活期间有效 (通常是栈上局部变量)。
///   因此移动被显式删除 —— 持有引用成员的类型移动语义本就危险。
class ScopedTimer
{
public:
  explicit ScopedTimer(double& out_ms) noexcept;
  ~ScopedTimer() noexcept;

  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;
  ScopedTimer(ScopedTimer&&) = delete;
  ScopedTimer& operator=(ScopedTimer&&) = delete;

  /// @brief 提前结束计时, 返回本次耗时 (ms) 并写入 out_ms。
  /// 重复调用返回第一次的结果, **不重复写** out_ms
  [[nodiscard]] double stop() noexcept;

private:
  double& out_ms_;
  Timer timer_;
  bool stopped_{false};
};

// ═══════════════════════════════════════════════════════════════════════════
// 2. 分模块耗时统计 (§7.3 第 2 条)
// ═══════════════════════════════════════════════════════════════════════════

/// @brief 单个阶段的耗时统计。
///
/// ⚠ count == 0 时 last/mean/min/max 均无意义 (皆为 0.0), 调用方需先判 count。
///
/// mean_ms 是**累计算术平均** (增量式更新), 不做滑动窗口 ——
/// 与 FpsCounter 的滑动窗口有意不同: 阶段耗时关心"整场跑下来这个模块慢不慢",
/// 而不是"最近几帧怎么样"。
struct LatencyStats {
  std::uint64_t count{0};   ///< 累计记录次数 (= 有效记录的帧数)
  double last_ms{0.0};      ///< 最近一次耗时
  double mean_ms{0.0};      ///< 累计算术平均
  double min_ms{0.0};       ///< 最小值 (count == 0 时无意义)
  double max_ms{0.0};       ///< 最大值 (count == 0 时无意义)
};

/// @brief 分阶段耗时统计表 —— 定长、热路径无堆分配、下标即身份。
///
/// 设计要点:
///   - 阶段在初始化期用 declare() 登记, 拿到一个**稳定下标**; 此后热路径只用下标,
///     不做字符串查找 (名字 → 下标的映射由调用方自己持有)。
///   - **frame_ms() 是当前帧**, 与 msg 的 stage_elapsed_ms 一一对应;
///     **stats() 是累计统计**, 供 /perception/timing 与 health_monitor 使用。
///
/// 典型用法:
/// @code
///   StageProfiler profiler;
///   const std::size_t kPre    = profiler.declare("preprocess");
///   const std::size_t kGround = profiler.declare("ground_segment");
///
///   // 每帧:
///   profiler.begin_frame();
///   double ms = 0.0;
///   { ScopedTimer st(ms); preprocess(cloud); }
///   profiler.record(kPre, ms);
/// @endcode
class StageProfiler
{
public:
  /// 阶段数上限。超出后 declare() 返回 kInvalidStage (调用方应据此在启动日志里报错)
  static constexpr std::size_t kMaxStages = 16;

  /// declare() 失败 / 下标非法的哨兵值
  static constexpr std::size_t kInvalidStage = std::numeric_limits<std::size_t>::max();

  /// @brief 声明一个阶段并返回其下标 (只应在初始化期调用)
  /// @return 0 .. kMaxStages-1; 已达上限 → kInvalidStage
  ///
  /// ⚠ **同名不去重**: 重名会各占一个下标。这是有意的 —— 去重会让两处不同逻辑
  ///   意外共享同一份统计, 静默且难查。需要查重的话调用方用 name() 自查即可
  ///   (只在初始化期做, 不在热路径上)。本函数会分配内存 (std::string) ——
  ///   名字存 std::string 而非 const char*, 是为了防止调用方传临时字符串造成悬垂。
  [[nodiscard]] std::size_t declare(const std::string& name);

  /// 已声明的阶段数
  [[nodiscard]] std::size_t size() const noexcept { return count_; }

  /// 下标是否有效 (即已被 declare 过)
  [[nodiscard]] bool valid_stage(std::size_t idx) const noexcept { return idx < count_; }

  /// @brief 阶段名; 下标非法 → 返回**静态**空字符串 (可安全长期持有其引用)
  [[nodiscard]] const std::string& name(std::size_t idx) const noexcept;

  /// @brief 记录一次耗时 (ms): 同时更新**累计统计**与**当前帧**。
  ///
  /// 脏数据静默丢弃 (见文件头代价契约第 3 条):
  ///   - 下标非法 → 丢弃
  ///   - ms < 0 / NaN / Inf → 丢弃 (一次 NaN 会让 min/max 永久污染)
  /// 丢弃时**不增加 count** —— 因此"某阶段 count 长期为 0"本身就是可观测的信号。
  void record(std::size_t idx, double ms) noexcept;

  /// 开始新一帧 (= 只清空当前帧耗时)。应在每帧管线入口调用一次
  void begin_frame() noexcept;

  /// @brief 当前帧各阶段耗时 (ms), 下标与 declare() 一致。
  /// ⚠ 数组定长 kMaxStages, 只有前 size() 个元素有意义 ——
  ///   薄壳拷进 msg 的 stage_elapsed_ms 时**只取前 size() 个**。
  /// 未 record 过的阶段为 0.0
  [[nodiscard]] const std::array<float, kMaxStages>& frame_ms() const noexcept
  {
    return frame_ms_;
  }

  /// @brief 累计统计; 下标非法 → 返回默认构造的 LatencyStats (count == 0)
  [[nodiscard]] LatencyStats stats(std::size_t idx) const noexcept;

  /// 清空所有**累计统计**(当前帧不受影响); 保留已声明的阶段与下标
  void reset_stats() noexcept;

private:
  struct Stage {
    std::string name;
    LatencyStats stats;
  };

  std::array<Stage, kMaxStages> stages_{};
  std::size_t count_{0};
  std::array<float, kMaxStages> frame_ms_{};
};

// ═══════════════════════════════════════════════════════════════════════════
// 3. 帧率计数器 (§7.3 第 3 条)
// ═══════════════════════════════════════════════════════════════════════════

/// @brief 帧率计数器 (滑动窗口平均)。
///
/// 只保留最近 window 个**帧间隔**; fps() 由维护好的累加和直接算出 (O(1)),
/// **不遍历窗口** —— 实现 tick()/reset() 时必须同步维护 interval_sum_ms_,
/// 否则 fps() 会静默漂移。
///
/// ⚠ 首帧只记时间戳、不产生间隔: 需 tick 满 2 次 fps() 才有值。
///
/// 时钟可注入 —— tick(now) 允许单测用假时间戳验证, 不必真 sleep。
class FpsCounter
{
public:
  /// @param window 滑动窗口长度 (帧间隔个数); 0 → 按 1 处理
  explicit FpsCounter(std::size_t window = 30);

  /// 记一帧 (取当前 steady_clock 时刻)
  void tick();

  /// 记一帧 (用调用方给的时间戳 —— 单测注入假时钟用)
  void tick(const SteadyClock::time_point& now);

  /// 滑动窗口内的平均帧率 (Hz); 样本不足 2 帧 → 0.0
  [[nodiscard]] double fps() const noexcept;

  /// 最近一帧与上一帧的间隔 (ms); 样本不足 2 帧 → 0.0
  [[nodiscard]] double last_interval_ms() const noexcept;

  /// 已累计的间隔样本数 (= tick 次数 - 1)
  [[nodiscard]] std::size_t sample_count() const noexcept { return intervals_ms_.size(); }

  /// 清空计时历史 (保留窗口长度); 下一帧重新当作首帧
  void reset() noexcept;

private:
  std::size_t window_{1};
  std::deque<double> intervals_ms_;
  double interval_sum_ms_{0.0};   ///< = sum(intervals_ms_); 与上面同步维护
  double last_interval_ms_{0.0};  ///< 缓存 intervals_ms_.back(), 供 noexcept 访问器
  std::optional<SteadyClock::time_point> last_;
};

// ═══════════════════════════════════════════════════════════════════════════
// 4. 端到端延迟告警判定 (§7.3 第 5 条)
// ═══════════════════════════════════════════════════════════════════════════

/// @brief 延迟等级。**只做判定** —— 打印/发布 ROS 话题由薄壳负责 (见文件头偏差 a)
enum class LatencyLevel : std::uint8_t {
  OK    = 0,   ///< 在预算内
  WARN  = 1,   ///< 超出警戒线 (任务书 §15.3 端到端预算 56ms)
  ERROR = 2,   ///< 超过硬线 (任务书 §7.3 "延迟超标 >80ms 自动告警")
};

/// 延迟判定阈值 (可由 yaml 覆盖; 默认值出处见各字段注释)
struct LatencyThresholds {
  double warn_ms{56.0};    ///< §15.3: max(雷达 34, 视觉 46) + 10 = 56ms
  double error_ms{80.0};   ///< §7.3: 端到端超标线
};

/// @brief 端到端耗时 → 告警等级。
///
///   end_to_end <= warn_ms                    → OK
///   warn_ms <  end_to_end <= error_ms        → WARN
///   end_to_end >  error_ms                   → ERROR
///
/// 边界处理 (两条, 都是防御性设计):
///   - 阈值非法 (warn_ms > error_ms, 例如 yaml 配反) → 退化为单一门限, 只按 error_ms 判
///   - 输入为非有限值 (NaN / ±Inf) → ERROR —— 异常数据不能掉进 OK
[[nodiscard]] LatencyLevel classify_latency(
    double end_to_end_ms,
    const LatencyThresholds& thresholds = LatencyThresholds());

/// 等级 → 字符串 (诊断输出 / 日志用)
[[nodiscard]] const char* to_string(LatencyLevel level) noexcept;

}  // namespace utils
}  // namespace br_perception
