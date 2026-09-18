#pragma once

/// ═══════════════════════════════════════════════════════════════════════════
/// protocol_decoder.hpp — 主控下行帧解析 (任务书 §5.2)
///
/// 与 encoder 是**一对**: 同一个帧格式, 但方向相反, 而且难度也相反 ——
/// encoder 拿的是结构化消息, decoder 拿的是**一串字节**:
///   分包 (一帧拆成几次读到) / 粘包 (一次读到多帧) / 前面有垃圾 / 中途丢字节,
///   全都要能扛住。所以它是**状态机**, 不是纯函数。
///
/// 帧格式与 encoder 完全一致 (帧头 0xAA 0x55 / 序号 / 类型 / uint16 长度 /
/// payload / CRC16 小端), 见 protocol_encoder.hpp 文件头。
///
/// 职责边界 (**刻意只做这些**):
///   · 帧同步 + 长度切分 + CRC 校验 —— 规格完整, 实现
///   · 0x10 BR_RESET 解析 —— 规格完整 (任务书 §5.1 v1.6 给了逐字节布局), 实现
///   · **主控侧其余类型 (ACK / 请求重发 / 切换模式 / 机器人状态) 一律不猜** ——
///     任务书只写"主控可能发送的内容", 没有字节级定义。原样交给调用方, 等协议冻结
///   · 发布到 `/communication/from_master` 是薄壳 (comm_node) 的事, 不在这里
///
/// ⚠ 复用 protocol_encoder.hpp 里的 `ProtocolConfig` 与 `Crc16` ——
///   收发两端必须逐位一致, 共用一份实现比各写一份安全。代价是本头文件也间接
///   引入了 ROS 消息头 (encoder 的输入契约是 PerceptionFrame)。
///
/// 线程安全: **非线程安全** (内部有接收缓冲与统计)。串口接收是单线程的,
///           由 serial_manager 的接收线程独占调用即可。
/// ═══════════════════════════════════════════════════════════════════════════

#include <cstddef>
#include <cstdint>
#include <vector>

#include "br_perception/communication/protocol_encoder.hpp"

namespace br_perception {
namespace communication {

/// 主控 → 感知 的包类型。
///
/// ⚠ 任务书 §5.2 只给了 0x10 的字节级定义, 其余 ("心跳回应/请求重发/切换模式/
///   机器人状态") 是列举而非规格 —— 故这里**只定义已确认的那一个**, 其余类型
///   原样以原始 type 字节交回调用方, 不预先造枚举。
constexpr std::uint8_t kMasterPacketBrReset = 0x10;

/// 0x10 BR_RESET 的 payload 长度: reset_zone_id(1) + reserve(2)
constexpr std::size_t kResetPayloadBytes = 3;

/// 已定义的 reset_zone_id 取值上限 (0 = 地面启动区, 1 = L1 重试区)
constexpr std::uint8_t kMaxKnownResetZone = 1;

/// 判定"主控重新编号了"所需的**连续递增 stale 帧**个数。
///
/// 背景: 任务书 §5.2 要求"帧号已更新则丢弃旧的"。但主控重启后从 0 重新编号时,
/// 旧规则会把 0..last_seq 这**整段新帧**都当成"旧的"丢掉 —— last_seq=175 时
/// 要白丢 176 帧 (50Hz 下约 3.5 s) 才恢复正常。
///
/// 区分依据 (三种"看起来旧"的帧, 只有第三种是重启):
///   · 重复帧        → 序号**不变**, 连串里全是同一个值
///   · 迟到的重传    → 序号**跳变**, 不与上一帧 stale 衔接
///   · 主控重启      → 0, 1, 2, ... **逐 1 递增**, 且**从 0 开始**
/// 所以判据同时要求"从 0 起"和"逐 1 递增"和"长度够" —— 三者都满足的误判面很小。
///
/// ⚠ 触发还额外要求 `last_seq_ >= 本阈值`: 否则 last_seq_ 本来就只有 2 时,
///   0,1,2 这串会把**重复帧** 2 也"恢复"成新帧, 反而违反 §5.2。
///   加上这一条, 恢复只在"确实能省下 ≥ 阈值 帧"时才发生。
constexpr std::uint8_t kRenumberRunThreshold = 3;

/// 一个**结构正确且 CRC 通过**的帧
struct DecodedFrame {
  std::uint8_t seq{0};              ///< 帧序号 (0-255)
  std::uint8_t type{0};             ///< 原始类型字节 (不预定义枚举, 见上)
  std::vector<std::uint8_t> payload;
};

/// 0x10 BR_RESET 的解析结果 (任务书 §5.1 v1.6)
///
/// 触发场景: 主控要求感知复位 —— 地面启动区重试, 或 BR 在 L1 重试区重试
/// (规则 5.1.3 / 5.3.2 的归还确认握手由感知侧的状态回置回应)。
struct ResetCommand {
  /// 0 = 地面启动区, 1 = L1 重试区
  std::uint8_t reset_zone_id{0};
};

/// @brief 流式帧解析器。
///
/// 用法: 串口每读到一段字节就 `feed()` 一次, 拿回本次解出的全部帧
/// (可能 0 个 —— 大多数调用都是 0 个, 因为一帧往往要分几次才读全)。
class ProtocolDecoder
{
public:
  /// @param config CRC / 容量上限参数, 必须与 encoder 用同一份
  ///
  /// payload 上界**默认从 config 派生** (ProtocolConfig::max_payload_bytes) ——
  /// 只有一个来源, 改 max_building_spots / max_obstacles 时自动跟随。
  explicit ProtocolDecoder(const ProtocolConfig& config = ProtocolConfig())
    : ProtocolDecoder(config, ProtocolConfig::max_payload_bytes(config))
  {
  }

  /// @brief 显式指定 payload 上界。
  /// **只有主控下行帧可能大于本 config 的上界时才需要** ——
  /// 例如将来加"期望路径下发"这类长帧。
  /// ⚠ 上界一旦小于某个合法帧, 那帧会被当成"假帧头"丢弃, 并**连带破坏后续同步**
  ///   (跳 1 字节后重新找 0xAA 0x55, 可能连环误判)。调下来容易, 收拾很难。
  ProtocolDecoder(const ProtocolConfig& config, std::size_t max_payload_bytes);

  /// 喂入一段收到的字节, 返回本次解出的完整帧 (按流中顺序)。
  /// len == 0 → 什么都不做; len > 0 但 data 为空 → 编程错误 (Debug 下 assert)
  ///
  /// ⚠ 返回 vector 意味着每次调用可能有一次堆分配。当前用途 (低频串口收帧,
  ///   一帧几十字节) 完全够用; 若将来发现它是热路径瓶颈, 可改为输出参数或回调 ——
  ///   那会破坏 API, 故把这个取舍现在写明, 免得将来重新推一遍。
  std::vector<DecodedFrame> feed(const std::uint8_t* data, std::size_t len);

  /// 便捷重载
  std::vector<DecodedFrame> feed(const std::vector<std::uint8_t>& data)
  {
    return feed(data.data(), data.size());
  }

  /// @brief 解析 0x10 BR_RESET 包。
  /// @return false = 类型不是 0x10 / payload 长度不为 3 / zone_id 取值未定义
  static bool parse_reset(const DecodedFrame& frame, ResetCommand& out);

  // ── 统计 (供 serial_manager / diagnostics 发布) ──

  std::uint64_t frames_ok() const noexcept { return frames_ok_; }
  std::uint64_t crc_errors() const noexcept { return crc_errors_; }

  /// 因**帧同步**而丢弃的字节数 (re-sync 时扫掉的垃圾)。
  /// ⚠ **不含**被判定为 stale 的整帧 —— 那个按**帧**计 (见 stale_frames)。
  /// 串口接错线 / 波特率不对时这个数会飞涨。
  std::uint64_t garbage_bytes() const noexcept { return garbage_bytes_; }

  /// 因序号不新而丢弃的帧数 —— 迟到的重传 / 重复帧
  std::uint64_t stale_frames() const noexcept { return stale_frames_; }

  /// 上述 stale 帧里**序号跨度有歧义**的那部分 (差值落在 [128, 255])。
  /// 非零 = 链路曾一次性丢过 ≥128 帧, 或主控重新编号 ——
  /// 因为 uint8 序号分不清"200 帧之后"和"56 帧之前"。正常应恒为 0。
  /// ⚠ 边界含 128: 该点两侧等距, 是最歧义的取值, 不是"还差一点"。
  std::uint64_t ambiguous_seq_frames() const noexcept { return ambiguous_seq_frames_; }

  /// 判定"主控重新编号"并自动恢复序号状态的次数 (见 kRenumberRunThreshold)。
  /// 非零 = 主控重启过。正常应恒为 0 —— 它不为 0 说明链路对面重新编过号,
  /// 值得看一眼是为什么 (真重启 / 还是本判据误触)。
  std::uint64_t renumber_recoveries() const noexcept { return renumber_recoveries_; }

  /// 只清零统计**计数器** (接收缓冲与序号状态保留)
  void reset_counters() noexcept;

  /// 只重置**链路状态**: 接收缓冲 + 序号, 计数器原样保留。
  ///
  /// 用于**链路重建** (fd 被 close+reopen 之后): 缓冲区里的半截帧必然是垃圾,
  /// 且对端可能已重新编号。与 reset() 的区别就是**不动计数器** ——
  /// 调用方 (serial_manager) 是拿 frames_ok() 直接赋值给累计统计的,
  /// 用 reset() 会让诊断里的 frames_received 归零, 看着像统计坏了。
  void reset_link_state() noexcept;

  /// 完全重置: 计数器 + 接收缓冲 + 序号状态。
  /// 用于**链路重建 / 主控重启** —— 主控若从 0 重新编号, 不重置会让前若干帧被误判 stale。
  void reset() noexcept;

  /// @brief 缓冲区里**等待更多输入**的字节数。
  /// 定义就是 `buffer_.size()` —— **含**"末尾孤立 0xAA, 可能是下一帧开头"这类残余;
  /// 不是"已找到帧头、只等 payload 补齐"的字节数。纯诊断, 不参与解码决策。
  std::size_t pending_bytes() const noexcept { return buffer_.size(); }

  /// 已接受的最后一个帧序号 (诊断用)
  std::uint8_t last_seq() const noexcept { return last_seq_; }

  /// 是否已经接受过至少一帧 (last_seq() 在此之前无意义)
  bool has_last_seq() const noexcept { return has_last_seq_; }

private:
  /// 在缓冲区里找第一个 0xAA 0x55; 找不到返回 kNoPos。
  ///
  /// **复杂度 O(buffer_.size())**, 且每轮 feed() 循环都从 buffer[0] 重扫,
  /// 最坏 O(n²)。当前可接受: max_payload_bytes_ 限制了 n 的上界, 单次 feed 的
  /// 输入通常只有几十字节。若将来在高带宽 + 严苛垃圾场景下发现瓶颈,
  /// 加一个"已确认无帧头的前缀"hint 即可, 不必改调用方。
  std::size_t find_header() const noexcept;

  /// 只清**序号状态** (序号 + 重新编号连串), 缓冲区与计数器都不碰。
  ///
  /// ⚠ 与 reset_link_state() 的区别在这里: 后者会 clear() 缓冲区, 那只在
  ///   "换 fd" 时才安全。判定"主控重新编号"是在**解析途中**发生的, 此刻缓冲区里
  ///   还躺着本次 feed() 尚未解析的后续帧 (粘包), 清掉就把它们一起丢了 ——
  ///   实测 20 帧一批会丢 17 帧。
  void clear_seq_state() noexcept;

  ProtocolConfig config_;
  Crc16 crc_;
  std::size_t max_payload_bytes_;

  std::vector<std::uint8_t> buffer_;

  /// 已接受的最后一个帧序号; 用于丢弃迟到的/重复的帧
  std::uint8_t last_seq_{0};
  bool has_last_seq_{false};

  /// 判定"主控重新编号"用的连串状态 (见 kRenumberRunThreshold 的说明)。
  /// ⚠ 任何一帧被**接受**都要清掉 —— 这个串必须连续, 中间夹一个正常帧就不算重启。
  struct RenumberRun {
    std::uint8_t length{0};        ///< 当前连串长度
    std::uint8_t last_seq{0};      ///< 连串里最后一个 stale 帧的 seq
    bool from_zero{false};         ///< 连串是否从 seq == 0 开始
  };
  RenumberRun renumber_run_{};

  std::uint64_t frames_ok_{0};
  std::uint64_t crc_errors_{0};
  std::uint64_t garbage_bytes_{0};
  std::uint64_t stale_frames_{0};
  std::uint64_t ambiguous_seq_frames_{0};
  std::uint64_t renumber_recoveries_{0};
};

}  // namespace communication
}  // namespace br_perception
