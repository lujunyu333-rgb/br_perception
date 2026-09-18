# 主控下行指令 —— 串口解出的原始帧 (任务书 §5.2 / §5.4)
#
# ⚠ 本消息**刻意不解释 payload 语义**。主控侧的帧类型除 0x10 BR_RESET 外
#   都还没有字节级定义 (任务书 §5.2 只列举了"可能发送的内容": ACK / 请求重发 /
#   切换模式 / 机器人状态 —— 一个字段都没给)。
#   与其替下位机猜一套布局, 不如把 seq / type / payload 原样透传,
#   由消费方按**已确认的部分**处理, 其余先记着。
#
# 消费方 (fusion_node) 目前只需要一件事: BR_RESET 事件 → 位姿复位。

std_msgs/Header header

# 主控侧的帧序号 (0-255, 与感知侧的上行序号是两套独立计数器)
uint8 seq

# 原始类型字节 (见下方常量)
uint8 type

# 原样透传的 payload (长度由原始帧的长度字段决定, 不做任何解释)
uint8[] payload

# ── 类型常量 (与 src/communication/protocol_decoder.hpp 的 kMasterPacketBrReset 对应) ──
# 0x10 主控要求感知复位。payload = [reset_zone_id, reserve, reserve],
# reset_zone_id: 0 = 地面启动区, 1 = L1 重试区 (任务书 §5.1 v1.6)
uint8 BR_RESET = 16
