#pragma once

/// ═══════════════════════════════════════════════════════════════════════════
/// comm_node.hpp — 通信薄壳节点 (任务书 §5.4)
///
/// 它是**薄壳**: 自己不实现任何协议或 I/O 逻辑, 只做四件事
///   1. 从 yaml/参数读配置, 填进 SerialConfig / ProtocolConfig
///   2. 订阅 `/perception/output` → `ProtocolEncoder` 编码 → `SerialManager` 入队
///   3. `SerialManager` 解出的下行帧 → 发布 `/communication/from_master`
///   4. 周期发布 `/communication/diagnostics`
///
/// 这正是设计文档 §4 硬性规则第 1 条想要的分工: 算法/协议类不碰 ROS,
/// 薄壳不认识算法 —— 换一串串口参数、换个话题名都不必动那三个模块。
///
/// ══ 线程 ══
///   本节点的方法会被**三条线程**调用, 每个方法下面各自标了约束:
///     · ROS executor 线程 —— on_perception / publish_diagnostics / send_debug_text
///     · SerialManager 接收线程 —— on_master_frame
///     · SerialManager 发送线程 —— 心跳 provider (只调 encoder_ 的 lambda)
///
///   ⚠ **本类假设单线程 executor** (rclcpp 默认)。当前实现没有任何成员级锁,
///     靠的是"**没有共享可变状态**": publish() 本身线程安全, ProtocolEncoder 的
///     序号是原子的, SerialManager 内部自有锁。新增计数成员时请用 std::atomic
///     (本文件已经这么做了), 免得换成 MultiThreadedExecutor 就出事。
/// ═══════════════════════════════════════════════════════════════════════════

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "br_perception/communication/protocol_encoder.hpp"
#include "br_perception/communication/serial_manager.hpp"
#include "br_perception/msg/master_command.hpp"
#include "br_perception/msg/perception_frame.hpp"

namespace br_perception {
namespace communication {

class CommNode : public rclcpp::Node
{
public:
  explicit CommNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// @brief 显式停串口收发。
  /// 虽然 SerialManager 是值成员、析构时也会 stop(), 但那样**晚于 ROS 基类析构** ——
  /// 线程回调可能碰到正在析构的 ROS 对象。显式在这里停, 不依赖"成员声明顺序恰好对"。
  ~CommNode() override;

  CommNode(const CommNode&) = delete;
  CommNode& operator=(const CommNode&) = delete;

private:
  /// @brief 读参数并返回完整配置。**必须能在成员初始化列表里调用** ——
  /// 所以它是静态的、接收 Node&, 而不是改 this 成员的实例方法。
  ///
  /// ⚠ 这是本类最容易写错的地方: 成员按**声明顺序**构造, 构造函数体里再
  ///   load_parameters() 已经太晚 —— serial_ 早就拿着默认配置构造完了,
  ///   yaml 里配的设备名永远到不了 SerialManager。必须走初始化列表。
  static SerialConfig load_serial_config(rclcpp::Node& node);

  /// ROS executor 线程: 一帧融合结果 → 编码 → 入发送队列。
  /// ⚠ 本回调**不在串口线程**; 但仍别做重活 —— 会拖住诊断定时器等其它 ROS 回调。
  void on_perception(const br_perception::msg::PerceptionFrame::SharedPtr msg);

  /// **SerialManager 接收线程**: 解出一帧主控指令 → 发布。
  ///
  /// ⚠ 策略: **一律原样发布, 不解析、不丢弃**。
  ///   `DecodedFrame{seq, type, payload}` 逐字段拷进 `MasterCommand`,
  ///   通信层不解释 payload 语义 —— 主控侧除 0x10 外都没有字节级定义,
  ///   "看不懂就丢"会把将来才定义的指令提前扔了。语义由消费方按已确认部分处理。
  ///
  /// ⚠ **在串口接收线程上** —— 任何阻塞都会卡住整个接收循环 (含后续帧的解码)。
  void on_master_frame(const DecodedFrame& frame);

  /// ROS executor 线程: 周期发布 /communication/diagnostics
  void publish_diagnostics();

  /// ROS executor 线程: 调试模式下额外发一行 ASCII 状态 (任务书 §5.4 的"调试模式")
  void send_debug_text();

  // ── 配置 (话题名等纯 ROS 侧的东西; 串口/协议配置见 serial_config_) ──
  std::string output_topic_{"/perception/output"};

  /// ⚠ 发布用**默认 QoS** (RELIABLE, depth 10)。若将来消费者用 BEST_EFFORT 订阅,
  ///   会**静默收不到** (ROS 2 最经典的坑, 本仓库已经踩过一次 QoS 不匹配)。
  ///   出现"话题在、发布在、就是收不到"时, 第一个怀疑 QoS。
  std::string from_master_topic_{"/communication/from_master"};
  std::string diagnostics_topic_{"/communication/diagnostics"};

  /// 诊断发布周期 [s]。构造时夹到 ≥0.1 —— 0 或负值会让 wall timer 变成忙循环。
  double diagnostics_interval_sec_{1.0};

  /// 任务书 §5.4 的"调试模式"开关: 比赛模式只发紧凑二进制帧; 调试模式额外发 ASCII 日志串。
  /// ⚠ **默认 false**。下位机侧对 0x7F 调试帧的处理没有规格, 比赛时务必关掉。
  /// ⚠ 调试帧与感知帧**共享发送队列与帧序号空间**:
  ///     · 共享队列 → 投递时若队列满, 挤掉的是感知帧。故本类只在队列空时才投调试帧。
  ///     · 共享序号 → 主控若按序号连续性判丢帧, 会因调试帧看到"感知帧跳号"。
  ///       这是协议未冻结项 (seq 是否跨 PacketType 共享), 确认后再定。
  bool debug_mode_{false};
  double debug_interval_sec_{2.0};   ///< 刻意与诊断周期不同, 免得两个 timer 挤在一起

  /// 串口 + 协议参数 (config/comm_params.yaml)。⚠ 构造顺序上它必须排在下面两个之前。
  SerialConfig serial_config_;

  // ⚠ 成员声明顺序 = 构造顺序: serial_config_ → encoder_ → serial_。
  //   encoder_ 显式吃 serial_config_.protocol —— 否则协议冻结后改了 CRC 参数,
  //   yaml 改了、SerialConfig 改了, 编码器却还用默认值 (CCITT-FALSE)。
  ProtocolEncoder encoder_;
  SerialManager serial_;

  /// 因超出协议上界被截断的融合帧数 (建筑位 >16 或障碍物 >10)。
  /// 这不是通信层的错, 但**只有通信层知道协议上界** —— 所以由它计数并上报,
  /// 让上层能发现"融合层输出了协议装不下的东西"。
  std::atomic<std::uint64_t> truncated_frames_{0};

  /// 串口始终打不开 (含重连失败) 时为 true —— 诊断里的 device_unavailable 标志
  std::atomic<bool> device_unavailable_{false};

  rclcpp::Subscription<br_perception::msg::PerceptionFrame>::SharedPtr perception_sub_;
  rclcpp::Publisher<br_perception::msg::MasterCommand>::SharedPtr from_master_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
};

}  // namespace communication
}  // namespace br_perception
