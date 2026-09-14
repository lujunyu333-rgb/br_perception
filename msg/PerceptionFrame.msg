# 感知帧消息
# 感知系统最终输出的消息格式，供通信/决策模块使用
# 每帧包含当前时刻完整的感知结果

# ── 时间戳 ──
builtin_interfaces/Time timestamp

# ── 自身位姿（世界坐标系）──
geometry_msgs/Pose2D ego_pose

# ── 里程计可靠性 (§9.1 v1.5 新增) ──
# Point-LIO 退化 / IMU 加速度方差过大 / 里程计超时 → false
# 主控收到 false 时应把所有 world 坐标的置信度整体降权 (× confidence_decay, 默认 0.5)
bool odometry_reliable

# ── 穆斯蒂卡状态 ──
# 状态码: 0=未知 1=柱上 2=TR转运中 3=BR持有 4=已供奉 5=掉落
uint8 mustika_state
# 穆斯蒂卡当前位置
geometry_msgs/Pose2D mustika_pose

# ── 穆斯蒂卡柱状态 ──
PillarStatus mustika_pillar

# ── 核心支柱状态 ──
PillarStatus core_pillar

# ── 建筑位列表 ──
BuildingSpot[] building_spots

# ── 障碍物列表 ──
Obstacle[] obstacles

# ── 诊断信息 ──
diagnostic_msgs/DiagnosticStatus[] diagnostics

# ── 帧序号（递增）──
uint32 frame_id
