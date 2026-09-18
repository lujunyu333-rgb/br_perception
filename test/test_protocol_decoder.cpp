/// ═══════════════════════════════════════════════════════════════════════════
/// test_protocol_decoder.cpp — 流式帧解析器单元测试 (任务书 §5.2 / §11.1)
///
/// ⚠ 本测试**不起 ROS 节点** —— decoder 不是 Node。
///
/// ⚠ 测试用的帧**由本文件手工拼装** (独立的位逐位 CRC), 不经过 ProtocolEncoder。
///   decoder 与 encoder 互为逆运算, 但如果用 encoder 造输入, 两边一起错就互相抵消、
///   测试照样通过 —— 那正是"循环验证"。这里刻意不共享任何打包代码。
///
/// 重点在**流式**这一面: 分包 / 粘包 / 前导垃圾 / 坏 CRC 重同步 / 序号门控。
/// ═══════════════════════════════════════════════════════════════════════════

#include "br_perception/communication/protocol_decoder.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace br_perception {
namespace communication {
namespace {

/// 独立于 Crc16 的**位逐位** CCITT-FALSE (与 encoder 测试里 golden 脚本同源)
std::uint16_t bitwise_ccitt_false(const std::vector<std::uint8_t>& data)
{
  std::uint16_t crc = 0xFFFF;
  for (const std::uint8_t b : data) {
    crc = static_cast<std::uint16_t>(crc ^ (static_cast<std::uint16_t>(b) << 8));
    for (int i = 0; i < 8; ++i) {
      crc = (crc & 0x8000u) ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021)
                            : static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

/// 手工拼一帧: 帧头 / 序号 / 类型 / uint16 长度(小端) / payload / CRC16(小端)
std::vector<std::uint8_t> build_frame(std::uint8_t seq,
                                      std::uint8_t type,
                                      const std::vector<std::uint8_t>& payload)
{
  std::vector<std::uint8_t> f = {
      0xAA, 0x55, seq, type,
      static_cast<std::uint8_t>(payload.size() & 0xFFu),
      static_cast<std::uint8_t>((payload.size() >> 8) & 0xFFu)};
  f.insert(f.end(), payload.begin(), payload.end());

  const std::uint16_t crc = bitwise_ccitt_false(f);
  f.push_back(static_cast<std::uint8_t>(crc & 0xFFu));
  f.push_back(static_cast<std::uint8_t>(crc >> 8));
  return f;
}

/// 一帧 BR_RESET 的 payload (zone_id + reserve×2)
std::vector<std::uint8_t> reset_payload(std::uint8_t zone)
{
  return {zone, 0x00, 0x00};
}

void append(std::vector<std::uint8_t>& dst, const std::vector<std::uint8_t>& src)
{
  dst.insert(dst.end(), src.begin(), src.end());
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// 基本切分: 整帧 / 分包 / 粘包
// ═══════════════════════════════════════════════════════════════════════════

TEST(DecoderFeedTest, 一次喂一个完整帧)
{
  ProtocolDecoder dec;
  const auto frames = dec.feed(build_frame(7, kMasterPacketBrReset, reset_payload(0)));

  ASSERT_EQ(1u, frames.size());
  EXPECT_EQ(7, frames[0].seq);
  EXPECT_EQ(0x10, frames[0].type);
  ASSERT_EQ(kResetPayloadBytes, frames[0].payload.size());
  EXPECT_EQ(0u, frames[0].payload[0]);

  EXPECT_EQ(1u, dec.frames_ok());
  EXPECT_EQ(0u, dec.crc_errors());
  EXPECT_EQ(0u, dec.garbage_bytes());
  EXPECT_EQ(0u, dec.pending_bytes());
}

TEST(DecoderFeedTest, 逐字节喂也能解出来)
{
  ProtocolDecoder dec;
  const auto bytes = build_frame(3, kMasterPacketBrReset, reset_payload(1));

  // 除最后 1 字节外, 每喂 1 字节都不该出帧
  for (std::size_t i = 0; i + 1 < bytes.size(); ++i) {
    EXPECT_TRUE(dec.feed(&bytes[i], 1).empty()) << "第 " << i << " 字节就出帧了";
    EXPECT_GT(dec.pending_bytes(), 0u);
  }

  const auto last = dec.feed(&bytes.back(), 1);
  ASSERT_EQ(1u, last.size());
  EXPECT_EQ(3, last[0].seq);
  EXPECT_EQ(0u, dec.pending_bytes());          // 全部消耗干净
}

TEST(DecoderFeedTest, 一帧分两次喂)
{
  ProtocolDecoder dec;
  const auto bytes = build_frame(9, kMasterPacketBrReset, reset_payload(1));
  const std::size_t half = bytes.size() / 2;

  EXPECT_TRUE(dec.feed(bytes.data(), half).empty());
  const auto frames = dec.feed(bytes.data() + half, bytes.size() - half);
  ASSERT_EQ(1u, frames.size());
  EXPECT_EQ(9, frames[0].seq);
}

TEST(DecoderFeedTest, 一次喂三帧全部解出)
{
  ProtocolDecoder dec;
  std::vector<std::uint8_t> stream;
  for (std::uint8_t s : {0u, 1u, 2u}) {
    append(stream, build_frame(s, kMasterPacketBrReset, reset_payload(s)));
  }

  const auto frames = dec.feed(stream);
  ASSERT_EQ(3u, frames.size());
  EXPECT_EQ(0, frames[0].seq);
  EXPECT_EQ(1, frames[1].seq);
  EXPECT_EQ(2, frames[2].seq);
  EXPECT_EQ(3u, dec.frames_ok());
  EXPECT_EQ(0u, dec.garbage_bytes());
}

TEST(DecoderFeedTest, 大payload帧)
{
  // 200 比典型帧 (28) 大得多, 但小于默认上限 —— 锁定的行为是"长度闸门不误伤合法大帧"。
  ProtocolDecoder dec;
  const std::vector<std::uint8_t> payload(200, 0xA5);
  const auto frames = dec.feed(build_frame(1, 0x7F, payload));

  ASSERT_EQ(1u, frames.size());
  ASSERT_EQ(200u, frames[0].payload.size());
  EXPECT_EQ(0xA5, frames[0].payload[199]);
}

TEST(DecoderFeedTest, 长度恰等于上限的帧被接受)
{
  // 边界"含"还是"不含"是几乎每个协议实现都会踩一次的坑, 这里一次钉死。
  // 上限值从 config 现算, 不写死 —— 改 max_building_spots 时本测试自动跟随。
  const std::size_t cap = ProtocolConfig::max_payload_bytes(ProtocolConfig{});
  ProtocolDecoder dec;
  const std::vector<std::uint8_t> payload(cap, 0xA5);

  EXPECT_EQ(1u, dec.feed(build_frame(1, 0x7F, payload)).size());
  EXPECT_EQ(0u, dec.garbage_bytes());
}

TEST(DecoderFeedTest, 长度超上限一字节即判假帧头)
{
  const std::size_t cap = ProtocolConfig::max_payload_bytes(ProtocolConfig{});
  ProtocolDecoder dec;
  const std::vector<std::uint8_t> payload(cap + 1, 0xA5);

  EXPECT_TRUE(dec.feed(build_frame(1, 0x7F, payload)).empty());
  EXPECT_GE(dec.garbage_bytes(), 1u);
}

TEST(DecoderFeedTest, 空输入什么都不做)
{
  ProtocolDecoder dec;
  EXPECT_TRUE(dec.feed(std::vector<std::uint8_t>{}).empty());
  EXPECT_EQ(0u, dec.pending_bytes());
  EXPECT_EQ(0u, dec.garbage_bytes());
}

TEST(DecoderFeedTest, 长度为0的空指针是合法输入)
{
  // "len == 0 配 nullptr" 是**合法空输入**, 不该 assert 也不该崩 ——
  // 这与 "len > 0 但 data 为空" 是两回事, 后者是编程错误 (Debug 下 assert)。
  // 这条把那个区分变成可执行契约。
  ProtocolDecoder dec;
  EXPECT_TRUE(dec.feed(nullptr, 0).empty());
  EXPECT_EQ(0u, dec.pending_bytes());
  EXPECT_EQ(0u, dec.garbage_bytes());
}

// ═══════════════════════════════════════════════════════════════════════════
// 帧同步
// ═══════════════════════════════════════════════════════════════════════════

TEST(DecoderSyncTest, 帧头前有垃圾字节)
{
  ProtocolDecoder dec;
  std::vector<std::uint8_t> stream = {0x00, 0xFF, 0x12, 0x34};
  append(stream, build_frame(1, kMasterPacketBrReset, reset_payload(0)));

  const auto frames = dec.feed(stream);
  ASSERT_EQ(1u, frames.size());
  EXPECT_EQ(1, frames[0].seq);
  EXPECT_EQ(4u, dec.garbage_bytes());
}

TEST(DecoderSyncTest, 末尾孤立的AA留到下一次)
{
  ProtocolDecoder dec;

  // 0x00 是垃圾; 末尾的 0xAA 可能是下一帧的开头, 必须留着
  EXPECT_TRUE(dec.feed(std::vector<std::uint8_t>{0x00, 0xAA}).empty());
  EXPECT_EQ(1u, dec.pending_bytes());
  EXPECT_EQ(1u, dec.garbage_bytes());
  // 计数之外, "缓冲里留的确实是 0xAA 而不是 0x00" 由下面那条
  // ASSERT_EQ(2, frames[0].seq) **隐式**证明: 留错字节就拼不出帧头, 解不出 seq=2。

  // 从帧头的第二个字节开始喂 (那个 AA 已经在缓冲区里了)
  const auto f = build_frame(2, kMasterPacketBrReset, reset_payload(0));
  const auto frames = dec.feed(std::vector<std::uint8_t>(f.begin() + 1, f.end()));
  ASSERT_EQ(1u, frames.size());
  EXPECT_EQ(2, frames[0].seq);
}

TEST(DecoderSyncTest, 连续两个AA后跟55)
{
  ProtocolDecoder dec;
  std::vector<std::uint8_t> stream = {0xAA};          // 多出来的一个 AA
  append(stream, build_frame(4, kMasterPacketBrReset, reset_payload(0)));

  const auto frames = dec.feed(stream);
  ASSERT_EQ(1u, frames.size());
  EXPECT_EQ(4, frames[0].seq);
  EXPECT_EQ(1u, dec.garbage_bytes());                 // 第一个 AA 被当垃圾跳过
}

TEST(DecoderSyncTest, 三帧之间夹垃圾)
{
  ProtocolDecoder dec;
  std::vector<std::uint8_t> stream = {0xDE, 0xAD};    // 前导垃圾
  for (std::uint8_t s : {10u, 11u, 12u}) {
    append(stream, build_frame(s, kMasterPacketBrReset, reset_payload(s)));
    stream.push_back(0xBE);                          // 帧间垃圾
  }

  const auto frames = dec.feed(stream);
  ASSERT_EQ(3u, frames.size());
  EXPECT_EQ(10, frames[0].seq);
  EXPECT_EQ(12, frames[2].seq);
  // 2 (前导) + 1 + 1 + 1 (每帧后的 0xBE)
  EXPECT_EQ(5u, dec.garbage_bytes());
}

// ═══════════════════════════════════════════════════════════════════════════
// 重同步
// ═══════════════════════════════════════════════════════════════════════════

TEST(DecoderResyncTest, CRC错后重新同步到下一帧)
{
  ProtocolDecoder dec;
  auto bad = build_frame(5, kMasterPacketBrReset, reset_payload(0));
  bad.back() ^= 0xFFu;                                // 破坏 CRC 高字节

  std::vector<std::uint8_t> stream = bad;
  append(stream, build_frame(6, kMasterPacketBrReset, reset_payload(1)));

  const auto frames = dec.feed(stream);
  ASSERT_EQ(1u, frames.size());                       // 坏帧丢弃, 好帧照解
  EXPECT_EQ(6, frames[0].seq);
  EXPECT_EQ(1u, dec.crc_errors());
  EXPECT_EQ(1u, dec.frames_ok());
}

TEST(DecoderResyncTest, 长度超上限被判为假帧头)
{
  ProtocolDecoder dec;
  // 长度字段 0xFFFF (65535) 远超默认上限 272 —— 垃圾里的假帧头
  std::vector<std::uint8_t> stream = {0xAA, 0x55, 0x01, 0x10, 0xFF, 0xFF};
  append(stream, build_frame(2, kMasterPacketBrReset, reset_payload(0)));

  const auto frames = dec.feed(stream);
  ASSERT_EQ(1u, frames.size());
  EXPECT_EQ(2, frames[0].seq);
  EXPECT_GE(dec.garbage_bytes(), 1u);
  EXPECT_EQ(0u, dec.crc_errors());                    // 长度闸门先拦下, 没走到 CRC
}

TEST(DecoderResyncTest, 只有半个帧头时不误判)
{
  ProtocolDecoder dec;
  // 0xAA 之后不是 0x55 → 整个都是垃圾, 不留残余
  EXPECT_TRUE(dec.feed(std::vector<std::uint8_t>{0xAA, 0x00, 0x11}).empty());
  EXPECT_EQ(0u, dec.pending_bytes());
  EXPECT_EQ(3u, dec.garbage_bytes());
}

// ═══════════════════════════════════════════════════════════════════════════
// 0x10 BR_RESET 解析 (任务书 §5.1 v1.6)
// ═══════════════════════════════════════════════════════════════════════════

TEST(ParseResetTest, 解析地面启动区与L1重试区)
{
  ProtocolDecoder dec;
  ResetCommand cmd{9};

  const auto f0 = dec.feed(build_frame(0, kMasterPacketBrReset, reset_payload(0)));
  ASSERT_EQ(1u, f0.size());
  ASSERT_TRUE(ProtocolDecoder::parse_reset(f0[0], cmd));
  EXPECT_EQ(0, cmd.reset_zone_id);

  // reserve 两个字节随便填, 不该影响解析
  const auto f1 = dec.feed(build_frame(1, kMasterPacketBrReset, {1, 0xAA, 0xBB}));
  ASSERT_EQ(1u, f1.size());
  ASSERT_TRUE(ProtocolDecoder::parse_reset(f1[0], cmd));
  EXPECT_EQ(1, cmd.reset_zone_id);

  // reserve 两字节必须**原样**留在 payload 里 —— 协议里它是"将来扩展位"。
  // 解码器若顺手把它们吃掉, 下位机将来启用扩展位时就是静默失联。
  ASSERT_EQ(kResetPayloadBytes, f1[0].payload.size());
  // 依据: 任务书 §5.1 v1.6 的 0x10 逐字节布局 = reset_zone_id / reserve×2 / CRC。
  //   payload[0] **就是 zone_id**, 喂进去的是 1, 解出来也是 1 —— 解码器只读不改,
  //   payload 原样交回调用方 (见下方"未定义的zone_id被拒绝但原值可见")。
  EXPECT_EQ(1, f1[0].payload[0]);
  EXPECT_EQ(0xAA, f1[0].payload[1]);
  EXPECT_EQ(0xBB, f1[0].payload[2]);
}

TEST(ParseResetTest, 类型或长度不符则拒绝且不修改out)
{
  ProtocolDecoder dec;
  ResetCommand cmd{9};

  const auto wrong_type = dec.feed(build_frame(0, 0x01, reset_payload(0)));
  ASSERT_EQ(1u, wrong_type.size());
  EXPECT_FALSE(ProtocolDecoder::parse_reset(wrong_type[0], cmd));

  const auto short_payload = dec.feed(build_frame(1, kMasterPacketBrReset, {0, 0}));
  ASSERT_EQ(1u, short_payload.size());
  EXPECT_FALSE(ProtocolDecoder::parse_reset(short_payload[0], cmd));

  const auto long_payload = dec.feed(build_frame(2, kMasterPacketBrReset, {0, 0, 0, 0}));
  ASSERT_EQ(1u, long_payload.size());
  EXPECT_FALSE(ProtocolDecoder::parse_reset(long_payload[0], cmd));

  EXPECT_EQ(9, cmd.reset_zone_id);                    // 三次拒绝都没动 out
}

TEST(ParseResetTest, 未定义的zone_id被拒绝但原值可见)
{
  ProtocolDecoder dec;
  const auto frames = dec.feed(build_frame(0, kMasterPacketBrReset, reset_payload(2)));
  ASSERT_EQ(1u, frames.size());

  ResetCommand cmd{9};
  EXPECT_FALSE(ProtocolDecoder::parse_reset(frames[0], cmd));
  EXPECT_EQ(2, frames[0].payload[0]);                 // 调用方仍能看到原始取值
  EXPECT_EQ(9, cmd.reset_zone_id);
}

// ═══════════════════════════════════════════════════════════════════════════
// 序号门控
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequenceGateTest, 序号回绕被接受)
{
  ProtocolDecoder dec;
  for (std::uint8_t s : {254u, 255u, 0u, 1u}) {
    const auto frames = dec.feed(build_frame(s, kMasterPacketBrReset, reset_payload(0)));
    ASSERT_EQ(1u, frames.size()) << "seq = " << static_cast<int>(s);
    EXPECT_EQ(s, frames[0].seq);
  }
  EXPECT_EQ(4u, dec.frames_ok());
  EXPECT_EQ(0u, dec.stale_frames());
}

TEST(SequenceGateTest, 迟到的旧帧被丢弃)
{
  ProtocolDecoder dec;
  ASSERT_EQ(1u, dec.feed(build_frame(5, kMasterPacketBrReset, reset_payload(0))).size());

  EXPECT_TRUE(dec.feed(build_frame(3, kMasterPacketBrReset, reset_payload(0))).empty());
  EXPECT_EQ(1u, dec.stale_frames());
  EXPECT_EQ(1u, dec.frames_ok());
  // ⚠ 虽然"只晚了 2 帧", 但 uint8 下它与"超前 254 帧"是**同一串字节** ——
  //   而后者正是"一次性丢了 ≥128 帧"这个本该报警的场景, 分不清就必须计入。
  //   依据: 本模块头文件对 ambiguous_seq_frames 的契约是"差值落在 [128,255]",
  //   而 stale 帧的无符号差值恒落在 [128,255] (差值 0 是重复帧, 另计)。
  //   (按"实际差 2 帧就不歧义"理解会漏掉 0→200 那类真丢帧, 见下面那条用例。)
  EXPECT_EQ(1u, dec.ambiguous_seq_frames());
}

TEST(SequenceGateTest, 重复帧被丢弃)
{
  ProtocolDecoder dec;
  ASSERT_EQ(1u, dec.feed(build_frame(7, kMasterPacketBrReset, reset_payload(0))).size());
  EXPECT_TRUE(dec.feed(build_frame(7, kMasterPacketBrReset, reset_payload(0))).empty());

  EXPECT_EQ(1u, dec.stale_frames());
  EXPECT_EQ(0u, dec.ambiguous_seq_frames());          // 差 0, 是重复不是歧义
}

TEST(SequenceGateTest, 序号跨度超过128时计入歧义)
{
  ProtocolDecoder dec;
  ASSERT_EQ(1u, dec.feed(build_frame(0, kMasterPacketBrReset, reset_payload(0))).size());

  // 丢了一整轮 (200 帧) 后序号跳到 200: uint8 分不清"晚 200 帧"和"早 56 帧",
  // 会被判为 stale —— 行为改不了, 但必须**可观测**。
  EXPECT_TRUE(dec.feed(build_frame(200, kMasterPacketBrReset, reset_payload(0))).empty());
  EXPECT_EQ(1u, dec.stale_frames());
  EXPECT_EQ(1u, dec.ambiguous_seq_frames());
}

// ── 128 窗口的三个临界点 ──
// 上面只测了"跳 200"这一侧。真正会被写错的是边界: 127 / 128 / 129 三个值
// 分别对应"窗口内算新"、"两侧等距最歧义"、"明确陈旧"。三条一起才把
// "相差 ≤ 128 帧"这条假设从注释变成可执行契约。

TEST(SequenceGateTest, 差值127仍在窗口内算新)
{
  ProtocolDecoder dec;
  ASSERT_EQ(1u, dec.feed(build_frame(0, kMasterPacketBrReset, reset_payload(0))).size());
  ASSERT_EQ(1u, dec.feed(build_frame(127, kMasterPacketBrReset, reset_payload(0))).size());

  EXPECT_EQ(2u, dec.frames_ok());
  EXPECT_EQ(0u, dec.stale_frames());
  EXPECT_EQ(0u, dec.ambiguous_seq_frames());
}

TEST(SequenceGateTest, 差值128判旧且计入歧义)
{
  // ⚠ 128 是**最歧义**的一点: "晚 128 帧"与"早 128 帧"等可能, 无法判断。
  //   行为上取保守 (判旧), 但必须计入歧义 —— 阈值写成 129 会正好漏掉它。
  ProtocolDecoder dec;
  ASSERT_EQ(1u, dec.feed(build_frame(0, kMasterPacketBrReset, reset_payload(0))).size());
  EXPECT_TRUE(dec.feed(build_frame(128, kMasterPacketBrReset, reset_payload(0))).empty());

  EXPECT_EQ(1u, dec.stale_frames());
  EXPECT_EQ(1u, dec.ambiguous_seq_frames());
}

TEST(SequenceGateTest, 差值129判旧且计入歧义)
{
  ProtocolDecoder dec;
  ASSERT_EQ(1u, dec.feed(build_frame(0, kMasterPacketBrReset, reset_payload(0))).size());
  EXPECT_TRUE(dec.feed(build_frame(129, kMasterPacketBrReset, reset_payload(0))).empty());

  EXPECT_EQ(1u, dec.stale_frames());
  EXPECT_EQ(1u, dec.ambiguous_seq_frames());
}

TEST(SequenceGateTest, 高序号跳到低序号但差在窗口内算新)
{
  // 250 → 3: 无符号差 = 9, 即"晚 9 帧" → 必须接受。
  // 这是回绕窗口的另一侧, 与"差值127"互为反向。
  ProtocolDecoder dec;
  ASSERT_EQ(1u, dec.feed(build_frame(250, kMasterPacketBrReset, reset_payload(0))).size());
  ASSERT_EQ(1u, dec.feed(build_frame(3, kMasterPacketBrReset, reset_payload(0))).size());

  EXPECT_EQ(0u, dec.stale_frames());
  EXPECT_EQ(0u, dec.ambiguous_seq_frames());
}

TEST(SequenceGateTest, 第一帧永远被接受)
{
  ProtocolDecoder dec;
  EXPECT_FALSE(dec.has_last_seq());
  // 首帧序号是 200 也不该被当成 stale —— 此前没有任何参照
  ASSERT_EQ(1u, dec.feed(build_frame(200, kMasterPacketBrReset, reset_payload(0))).size());
  EXPECT_TRUE(dec.has_last_seq());
  EXPECT_EQ(200, dec.last_seq());
}

// ═══════════════════════════════════════════════════════════════════════════
// 主控重新编号 (自动恢复)
//
// ⚠ 判据必须**同时**看"从 0 起"和"逐 1 递增", 不能只看连串长度 —— 主控重传一个
//   窗口 (10,11,12) 同样递增且连续, 只看长度就会把本该丢弃的旧帧恢复成新帧,
//   那直接违反任务书 §5.2「帧号已更新则丢弃旧的」。下面三条负例就是钉这个的。
// ═══════════════════════════════════════════════════════════════════════════

TEST(RenumberTest, 主控从零重新编号后自动恢复)
{
  ProtocolDecoder dec;
  // 先跑到 100: 此时 seq=0 相对 100 被判陈旧 (判据: (256-last_seq_) >= 128),
  // 而按旧规则要一路白丢到 seq=101 才自然恢复 —— 正是本判据要解决的场景
  ASSERT_EQ(1u, dec.feed(build_frame(100, kMasterPacketBrReset, reset_payload(0))).size());

  // 主控重启从 0 重新编号: 0 / 1 仍按旧规则丢弃
  EXPECT_TRUE(dec.feed(build_frame(0, kMasterPacketBrReset, reset_payload(0))).empty());
  EXPECT_TRUE(dec.feed(build_frame(1, kMasterPacketBrReset, reset_payload(0))).empty());
  EXPECT_EQ(2u, dec.stale_frames());
  EXPECT_EQ(0u, dec.renumber_recoveries());

  // 第 3 帧凑满连串 → 判定重新编号, 且**这一帧本身**按新编号接受
  const auto f = dec.feed(build_frame(2, kMasterPacketBrReset, reset_payload(0)));
  ASSERT_EQ(1u, f.size());
  EXPECT_EQ(2, f[0].seq);
  EXPECT_EQ(1u, dec.renumber_recoveries());
  EXPECT_EQ(2u, dec.frames_ok());        // 100 那帧 + 恢复回来的 2
  EXPECT_EQ(2u, dec.stale_frames());     // 恢复帧不计入 stale, 仍是 0/1 两帧
  EXPECT_EQ(2, dec.last_seq());

  // 恢复后按新编号正常推进
  ASSERT_EQ(1u, dec.feed(build_frame(3, kMasterPacketBrReset, reset_payload(0))).size());
  EXPECT_EQ(2u, dec.stale_frames());
}

TEST(RenumberTest, 粘包时恢复不能丢掉同批次的后续帧)
{
  // ⚠ 恢复发生在**解析途中**, 而那些字节可能是一批读进来的 (粘包), 缓冲区里
  //   还躺着本次 feed() 尚未处理的后续帧。恢复若顺手清了缓冲区 (例如图省事调
  //   reset_link_state), 这批剩下的帧会连同字节一起消失 —— 而且 CRC / 垃圾字节
  //   两个计数都正常, 完全静默。实测 20 帧一批会丢 17 帧, 单测每帧单独喂时抓不到。
  ProtocolDecoder dec;
  ASSERT_EQ(1u, dec.feed(build_frame(100, kMasterPacketBrReset, reset_payload(0))).size());

  std::vector<std::uint8_t> batch;
  for (std::uint8_t s : {0u, 1u, 2u, 3u, 4u}) {
    append(batch, build_frame(s, kMasterPacketBrReset, reset_payload(0)));
  }

  const auto frames = dec.feed(batch);
  // 0/1 判陈旧丢弃; 2 触发恢复并被接受; 3/4 必须照常解出
  ASSERT_EQ(3u, frames.size());
  EXPECT_EQ(2, frames[0].seq);
  EXPECT_EQ(3, frames[1].seq);
  EXPECT_EQ(4, frames[2].seq);
  EXPECT_EQ(1u, dec.renumber_recoveries());
  EXPECT_EQ(2u, dec.stale_frames());
  EXPECT_EQ(4u, dec.frames_ok());          // 100 + 2 + 3 + 4
  EXPECT_EQ(0u, dec.pending_bytes());      // 本批全部消耗干净, 没被吞掉
}

TEST(RenumberTest, 任意序号的递增连串不算重新编号)
{
  ProtocolDecoder dec;
  ASSERT_EQ(1u, dec.feed(build_frame(100, kMasterPacketBrReset, reset_payload(0))).size());

  // 主控重传一个窗口 (10,11,12): 递增且连续, 但**不从 0 起** → 必须全部丢弃
  for (std::uint8_t s : {10u, 11u, 12u}) {
    EXPECT_TRUE(dec.feed(build_frame(s, kMasterPacketBrReset, reset_payload(0))).empty())
        << "seq = " << static_cast<int>(s);
  }
  EXPECT_EQ(3u, dec.stale_frames());
  EXPECT_EQ(0u, dec.renumber_recoveries());
  EXPECT_EQ(1u, dec.frames_ok());
}

TEST(RenumberTest, 重复帧连串不算重新编号)
{
  ProtocolDecoder dec;
  ASSERT_EQ(1u, dec.feed(build_frame(5, kMasterPacketBrReset, reset_payload(0))).size());

  // 同一序号连发: 序号**不变**, 永远凑不成"逐 1 递增"
  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(dec.feed(build_frame(5, kMasterPacketBrReset, reset_payload(0))).empty());
  }
  EXPECT_EQ(4u, dec.stale_frames());
  EXPECT_EQ(0u, dec.renumber_recoveries());
}

TEST(RenumberTest, 序号很小时不恢复以免把重复帧当新帧)
{
  ProtocolDecoder dec;
  ASSERT_EQ(1u, dec.feed(build_frame(2, kMasterPacketBrReset, reset_payload(0))).size());

  // 0/1 像新编号的开头, 但 2 是**已经收过的重复帧**。此时按旧规则再等一帧 (seq=3)
  // 就自然恢复, 判据没必要出手 —— 出手反而会把重复帧 2 恢复成新帧, 违反 §5.2。
  EXPECT_TRUE(dec.feed(build_frame(0, kMasterPacketBrReset, reset_payload(0))).empty());
  EXPECT_TRUE(dec.feed(build_frame(1, kMasterPacketBrReset, reset_payload(0))).empty());
  EXPECT_TRUE(dec.feed(build_frame(2, kMasterPacketBrReset, reset_payload(0))).empty());
  EXPECT_EQ(0u, dec.renumber_recoveries());
  EXPECT_EQ(3u, dec.stale_frames());

  // 自然恢复: 3 比 2 新, 直接接受
  ASSERT_EQ(1u, dec.feed(build_frame(3, kMasterPacketBrReset, reset_payload(0))).size());
  EXPECT_EQ(2u, dec.frames_ok());
}

TEST(RenumberTest, 恢复后连串被清空不会连续触发)
{
  ProtocolDecoder dec;
  ASSERT_EQ(1u, dec.feed(build_frame(100, kMasterPacketBrReset, reset_payload(0))).size());
  dec.feed(build_frame(0, kMasterPacketBrReset, reset_payload(0)));
  dec.feed(build_frame(1, kMasterPacketBrReset, reset_payload(0)));
  ASSERT_EQ(1u, dec.feed(build_frame(2, kMasterPacketBrReset, reset_payload(0))).size());
  ASSERT_EQ(1u, dec.renumber_recoveries());

  // 恢复后 last_seq_=2。再来一帧 1 (落后于 2) 只是普通 stale, 不该再次恢复
  EXPECT_TRUE(dec.feed(build_frame(1, kMasterPacketBrReset, reset_payload(0))).empty());
  EXPECT_EQ(1u, dec.renumber_recoveries());
  EXPECT_EQ(3u, dec.stale_frames());
}

// ═══════════════════════════════════════════════════════════════════════════
// 重置
// ═══════════════════════════════════════════════════════════════════════════

TEST(ResetTest, reset清空计数器缓冲与序号状态)
{
  ProtocolDecoder dec;
  dec.feed(build_frame(200, kMasterPacketBrReset, reset_payload(0)));
  ASSERT_TRUE(dec.has_last_seq());

  dec.reset();
  EXPECT_FALSE(dec.has_last_seq());
  EXPECT_EQ(0u, dec.pending_bytes());
  EXPECT_EQ(0u, dec.frames_ok());
  EXPECT_EQ(0u, dec.stale_frames());
  EXPECT_EQ(0u, dec.crc_errors());
  EXPECT_EQ(0u, dec.garbage_bytes());
  EXPECT_EQ(0u, dec.ambiguous_seq_frames());

  // 重置后 0 号被当作新帧接受 —— 这正是"主控重启重新编号"要的行为
  ASSERT_EQ(1u, dec.feed(build_frame(0, kMasterPacketBrReset, reset_payload(0))).size());
}

TEST(ResetTest, reset_counters只清计数器不动序号状态)
{
  ProtocolDecoder dec;
  dec.feed(build_frame(5, kMasterPacketBrReset, reset_payload(0)));
  dec.reset_counters();

  EXPECT_EQ(0u, dec.frames_ok());
  EXPECT_TRUE(dec.has_last_seq());                    // 序号状态保留
  EXPECT_EQ(5, dec.last_seq());

  // 旧帧仍被判 stale —— 证明状态真的没被清
  EXPECT_TRUE(dec.feed(build_frame(3, kMasterPacketBrReset, reset_payload(0))).empty());
  EXPECT_EQ(1u, dec.stale_frames());
}

TEST(ResetTest, reset_link_state只清链路状态保留计数器)
{
  ProtocolDecoder dec;
  ASSERT_EQ(1u, dec.feed(build_frame(5, kMasterPacketBrReset, reset_payload(0))).size());
  EXPECT_TRUE(dec.feed(build_frame(3, kMasterPacketBrReset, reset_payload(0))).empty());
  ASSERT_EQ(1u, dec.frames_ok());
  ASSERT_EQ(1u, dec.stale_frames());

  // 缓冲里塞半截帧 —— 换 fd 之后它必然是垃圾 (新链路的字节从帧头重新开始)
  const auto half = build_frame(9, kMasterPacketBrReset, reset_payload(0));
  ASSERT_TRUE(dec.feed(half.data(), half.size() - 2).empty());
  ASSERT_GT(dec.pending_bytes(), 0u);

  dec.reset_link_state();

  // 链路状态清了……
  EXPECT_FALSE(dec.has_last_seq());
  EXPECT_EQ(0u, dec.pending_bytes());
  // ……但计数器**原样保留**: serial_manager 拿 frames_ok() 直接赋值给累计统计,
  //   这里若归零, 诊断里的 frames_received 会莫名其妙掉回去。
  EXPECT_EQ(1u, dec.frames_ok());
  EXPECT_EQ(1u, dec.stale_frames());

  // 序号状态确实清了: 0 号重新被接受 (重连后对端可能从 0 重新编号)
  ASSERT_EQ(1u, dec.feed(build_frame(0, kMasterPacketBrReset, reset_payload(0))).size());
  EXPECT_EQ(2u, dec.frames_ok());
}

TEST(ResetTest, 重置前的半截帧不会污染重置后的解析)
{
  ProtocolDecoder dec;
  const auto f = build_frame(1, kMasterPacketBrReset, reset_payload(0));
  ASSERT_TRUE(dec.feed(f.data(), f.size() - 2).empty());   // 差 2 字节, 卡在缓冲里
  EXPECT_GT(dec.pending_bytes(), 0u);

  dec.reset();
  EXPECT_EQ(0u, dec.pending_bytes());

  // 重新喂一整帧, 必须干净解出
  const auto frames = dec.feed(build_frame(0, kMasterPacketBrReset, reset_payload(0)));
  ASSERT_EQ(1u, frames.size());
  EXPECT_EQ(0, frames[0].seq);
}

// ═══════════════════════════════════════════════════════════════════════════
// 与 encoder 的互操作
//
// 上面所有用例的手工拼帧与 ProtocolEncoder 是**两份独立实现**。
//   · encoder 的正确性 → 由它自己那份独立 golden 帧钉死
//   · decoder 的正确性 → 由本文件的用例钉死
//   但 **两者彼此是否兼容**, 上面没有任何地方验证过 —— 协议改了其中一边,
//   另一边会静默不匹配, 直到联调才暴露。
// 下面这条把环闭上。注意它**不是**循环验证: golden 是外部参照, 它验证的是
// "两份独立实现遵守同一份协议"。
// ═══════════════════════════════════════════════════════════════════════════

TEST(RoundTripTest, 编码器输出必须能被自己的解码器解出)
{
  ProtocolEncoder enc;
  const br_perception::msg::PerceptionFrame f;     // 全默认: 0 建筑位 / 0 障碍物
  const EncodeResult r = enc.encode(f);

  ProtocolDecoder dec;
  const auto frames = dec.feed(r.bytes);

  ASSERT_EQ(1u, frames.size());
  EXPECT_EQ(0, frames[0].seq);
  EXPECT_EQ(0x01, frames[0].type);                 // PacketType::PERCEPTION
  // payload = 头部 16 + 穆斯蒂卡 8 + 柱子 2 + 建筑位计数 1 + 障碍物计数 1
  EXPECT_EQ(28u, frames[0].payload.size());
  EXPECT_EQ(0u, dec.crc_errors());
  EXPECT_EQ(0u, dec.garbage_bytes());
  EXPECT_EQ(0u, dec.pending_bytes());
}

TEST(RoundTripTest, 连续多帧编码后一次喂给解码器)
{
  ProtocolEncoder enc;
  const br_perception::msg::PerceptionFrame f;

  std::vector<std::uint8_t> stream;
  for (int i = 0; i < 3; ++i) {
    append(stream, enc.encode(f).bytes);
  }

  ProtocolDecoder dec;
  const auto frames = dec.feed(stream);
  ASSERT_EQ(3u, frames.size());
  EXPECT_EQ(0, frames[0].seq);
  EXPECT_EQ(1, frames[1].seq);
  EXPECT_EQ(2, frames[2].seq);
  EXPECT_EQ(0u, dec.stale_frames());               // 序号连续, 无丢帧
}

}  // namespace communication
}  // namespace br_perception
