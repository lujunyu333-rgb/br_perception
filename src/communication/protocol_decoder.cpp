#include "br_perception/communication/protocol_decoder.hpp"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace br_perception {
namespace communication {

namespace {

constexpr std::size_t kNoPos = static_cast<std::size_t>(-1);

/// 序号比较, 处理 255→0 回绕: a 是否比 b **更新** (RFC 1982 序列号算术)。
///
/// ⚠ **前提: 两序号之差 ≤ 128 帧**。uint8 序号本身分不清"晚 200 帧"和"早 56 帧" ——
///   一次性丢 ≥129 帧时, 新帧会被误判成 stale 并持续丢弃, 直到差值回绕进 [1,127]
///   (最坏约 57 帧 ≈ 0.6s)。这不是崩溃, 但会**静默丢一段状态**, 故 feed() 里对
///   这种歧义计数 (ambiguous_seq_frames), 让它至少可观测。
bool seq_is_newer(std::uint8_t a, std::uint8_t b)
{
  return static_cast<std::int8_t>(static_cast<std::uint8_t>(a - b)) > 0;
}

}  // namespace

ProtocolDecoder::ProtocolDecoder(const ProtocolConfig& config,
                                 std::size_t max_payload_bytes)
  : config_(config)
  , crc_(config)
  , max_payload_bytes_(max_payload_bytes)
{
}

std::size_t ProtocolDecoder::find_header() const noexcept
{
  if (buffer_.size() < 2) {
    return kNoPos;
  }
  for (std::size_t i = 0; i + 1 < buffer_.size(); ++i) {
    if (buffer_[i] == kFrameHeader0 && buffer_[i + 1] == kFrameHeader1) {
      return i;
    }
  }
  return kNoPos;
}

std::vector<DecodedFrame> ProtocolDecoder::feed(const std::uint8_t* data,
                                                std::size_t len)
{
  std::vector<DecodedFrame> out;

  if (len == 0) {
    return out;                         // 合法输入: 空 = 什么都没发生
  }
  if (data == nullptr) {
    // 这是**编程错误**(有长度的空指针), 不是数据问题。Debug 下当场抓住;
    // Release 下安静返回 —— 但绝不假装"读到了一段数据"。
    assert(false && "ProtocolDecoder::feed(): len > 0 但 data 为空指针");
    return out;
  }
  buffer_.insert(buffer_.end(), data, data + len);

  for (;;) {
    // ── 1) 帧同步: 定位第一个 0xAA 0x55 ──
    const std::size_t header = find_header();

    if (header == kNoPos) {
      // 缓冲区里没有完整帧头。**末尾孤立的一个 0xAA 可能是下一帧的开头**,
      // 留着; 其余全是垃圾字节。
      const std::size_t keep = (!buffer_.empty() && buffer_.back() == kFrameHeader0) ? 1u : 0u;
      garbage_bytes_ += buffer_.size() - keep;
      buffer_.erase(buffer_.begin(),
                    buffer_.end() - static_cast<std::ptrdiff_t>(keep));
      break;
    }

    if (header > 0) {
      garbage_bytes_ += header;
      buffer_.erase(buffer_.begin(),
                    buffer_.begin() + static_cast<std::ptrdiff_t>(header));
    }

    // ── 2) 长度字段读得到吗 ──
    if (buffer_.size() < kFrameOverheadBytes) {
      break;                              // 还没收全帧头, 等下一段
    }
    const std::uint16_t payload_len = static_cast<std::uint16_t>(
        buffer_[4] | (static_cast<std::uint16_t>(buffer_[5]) << 8));

    // ── 3) 长度合法性: 超上限 → 这是垃圾里的假帧头, 跳过 1 字节重新同步 ──
    //       没有这道闸门, 一个坏长度字段就能让缓冲区无限增长。
    if (payload_len > max_payload_bytes_) {
      ++garbage_bytes_;
      buffer_.erase(buffer_.begin(), buffer_.begin() + 1);
      continue;
    }

    const std::size_t total = kFrameOverheadBytes + payload_len + kCrcBytes;
    if (buffer_.size() < total) {
      break;                              // 帧还没收全, 等下一段
    }

    // ── 4) CRC 校验 (覆盖除 CRC 自身外的整帧, 与 encoder 一致) ──
    const std::uint16_t expect = crc_.compute(buffer_.data(), total - kCrcBytes);
    const std::uint16_t actual = static_cast<std::uint16_t>(
        buffer_[total - 2] | (static_cast<std::uint16_t>(buffer_[total - 1]) << 8));

    if (expect != actual) {
      ++crc_errors_;
      // 只跳过 **1 字节**, 不是整帧: 我们无法证明这个帧头是真的 ——
      // 垃圾数据里同样可能出现 0xAA 0x55。跳 1 保证不漏掉真正的帧起始。
      //
      // 代价 (明确接受): 若垃圾里反复出现"像帧头 + 长度合法 + CRC 不符"的模式,
      // 每轮只前进 1 字节并重扫, 最坏 O(n²)。当前 n 有上界且输入很小, 可接受。
      // 换成"跳过 2 字节越过 AA 55"能省掉重复撞同一个假帧头, 但那要求协议保证
      // 帧头唯一性 —— 未冻结, 不赌。
      ++garbage_bytes_;
      buffer_.erase(buffer_.begin(), buffer_.begin() + 1);
      continue;
    }

    // ── 5) 成帧 ──
    DecodedFrame frame;
    frame.seq = buffer_[2];
    frame.type = buffer_[3];
    frame.payload.assign(
        buffer_.begin() + static_cast<std::ptrdiff_t>(kFrameOverheadBytes),
        buffer_.begin() + static_cast<std::ptrdiff_t>(kFrameOverheadBytes + payload_len));
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total));

    // 丢弃迟到的 / 重复的帧: 重传机制下旧帧可能后到, 执行它会把新状态覆盖回去。
    // ⚠ 副作用 (已知且有界): 主控若重启并从 0 重新编号, 会丢掉 0..last_seq 这一小段,
    //   随后自动恢复。要主动避开, 链路重建时调 reset()。
    if (has_last_seq_ && !seq_is_newer(frame.seq, last_seq_)) {
      ++stale_frames_;
      // 跨度过大 → uint8 序号分不清新旧, 计数以备诊断。
      // ⚠ 阈值是 **128** 不是 129: 差值 128 时"晚 128 帧"与"早 128 帧"等可能,
      //   恰恰是**最歧义**的那一点。写成 129 会把它漏掉。
      const std::uint8_t diff = static_cast<std::uint8_t>(frame.seq - last_seq_);
      if (diff >= 128u) {
        ++ambiguous_seq_frames_;
      }
      continue;
    }
    last_seq_ = frame.seq;
    has_last_seq_ = true;

    ++frames_ok_;
    out.push_back(std::move(frame));
  }

  return out;
}

bool ProtocolDecoder::parse_reset(const DecodedFrame& frame, ResetCommand& out)
{
  if (frame.type != kMasterPacketBrReset) {
    return false;
  }
  // 任务书 §5.1 v1.6: 帧号 / 0x10 / 长度=3 / reset_zone_id / reserve×2 / CRC
  // (payload 只有后 3 个字节, 长度字段的 3 指的正是它)
  if (frame.payload.size() != kResetPayloadBytes) {
    return false;
  }
  // 只认已定义的 0(地面启动区) / 1(L1 重试区)。未知取值的语义未冻结 ——
  // 与其猜, 不如让调用方从 frame.payload[0] 看到原值后自己决定。
  if (frame.payload[0] > kMaxKnownResetZone) {
    return false;
  }
  out.reset_zone_id = frame.payload[0];
  return true;
}

void ProtocolDecoder::reset_counters() noexcept
{
  frames_ok_ = 0;
  crc_errors_ = 0;
  garbage_bytes_ = 0;
  stale_frames_ = 0;
  ambiguous_seq_frames_ = 0;
}

void ProtocolDecoder::reset() noexcept
{
  reset_counters();
  buffer_.clear();
  last_seq_ = 0;
  has_last_seq_ = false;
}

}  // namespace communication
}  // namespace br_perception
