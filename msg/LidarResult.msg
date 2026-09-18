# 雷达感知结果 —— 容器 A(雷达流水线) → 容器 C(融合) 的正式接口 (设计文档 §5.1)
#
# 替代原先"拿 diagnostic_msgs/DiagnosticArray 当数据接口"的做法: 那是给人看的。
# MarkerArray / debug_cloud 保留, 但从此只作可视化, 不再承担数据职责。

# frame_id = "world", 带时间戳
std_msgs/Header header

# ── 检测结果 ──
Obstacle[] obstacles
BuildingSpot[] building_spots
PillarStatus mustika_pillar
PillarStatus core_pillar

# ── 诊断 ──
# 各阶段耗时 [ms], 下标与 StageProfiler::declare 的顺序一一对应 (timer_utils)
float32[] stage_elapsed_ms
# 雷达流水线在线 (替代原先塞在 DiagnosticArray 里的 watchdog 位)
bool watchdog_ok
