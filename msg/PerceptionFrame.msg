# 感知帧 —— 融合层最终输出, 经通信层编码后发给 STM32 主控 (设计文档 §5.2)
# 字段顺序按协议 §5.1 的分组排列, 便于与 protocol_encoder 逐段对照。

# frame_id = "world"。原先用裸 builtin_interfaces/Time, 丢了坐标系 —— 下游无从判断
std_msgs/Header header

# 自身位姿 (world 系, 2D)。
# 刻意保持 Pose2D: 机器人在哪一层由任务状态单独给出, 位姿本身不需要 z。
geometry_msgs/Pose2D ego_pose

# 里程计可靠性 (§9.1 v1.5): Point-LIO 退化 / IMU 振动 / 超时 → false。
# 主控收到 false 应把所有 world 坐标整体降权 (× confidence_decay, 默认 0.5)
bool odometry_reliable

# ── 比赛状态 (协议 §5.1 v1.6 组) ──
# 剩余比赛秒数 0~180 (感知自计时)
uint8 match_remaining_sec
# 被持握方块总数
uint8 held_object_count
# 0/1 持握超限 (规则 4.4.2: BR 限持 2 块)
uint8 holding_over_limit
# 0/1 终场释放告警 (规则 8.6.1: 蜂鸣时仍手持 = 该物体 0 分)
uint8 match_end_release_warning

# ── 穆斯蒂卡 ──
# 0=未知 1=柱上 2=TR转运中 3=BR持有 4=已供奉 5=掉落 6=重试归还中
# 6 为 v1.6 新增: 感知置 6 → 主控开始重试流程; 看到穆斯蒂卡回到柱上置回 1,
# 该变化即主控的"归还完成"信号 (规则 5.1.3/5.3.2)
uint8 mustika_state
# 0/1 圣所使命达成 (规则 4.5.1: ≥2 完整塔且 ≥1 在共享区) —— TR 取穆斯蒂卡的前置。
# 达成后**永久有效**, 塔之后被对手破坏也不复位 (规则 3.6.2)
uint8 sanctuary_mandate_fulfilled
# 穆斯蒂卡位置。必须是 PoseStamped 而非 Pose2D: 靠 z 区分"在柱上"还是"被拿起"
geometry_msgs/PoseStamped mustika_pose

# ── 柱子 / 建筑位 / 障碍物 ──
PillarStatus mustika_pillar
PillarStatus core_pillar
BuildingSpot[] building_spots
Obstacle[] obstacles

# ── 诊断 ──
diagnostic_msgs/DiagnosticStatus[] diagnostics

# 帧序号 (递增)。原名 frame_id 与 Header.frame_id 语义撞名 (一个是序号一个是坐标系), 故改名
uint32 seq
