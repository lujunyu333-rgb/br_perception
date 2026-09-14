# 穆斯蒂卡状态消息
# 描述穆斯蒂卡（比赛核心道具）的当前状态、位置和检测信息

# 状态码
# 0 = 未知 (UNKNOWN)
# 1 = 柱上 (ON_PILLAR)
# 2 = TR转运中 (IN_TRANSIT_TR)
# 3 = BR持有 (HELD_BY_BR)
# 4 = 已供奉 (OFFERED)
# 5 = 掉落 (DROPPED)
uint8 state

# 当前位置（世界坐标系）
geometry_msgs/Point position

# 检测来源
# 0 = 雷达 (LIDAR)
# 1 = 相机 (CAMERA)
# 2 = 融合 (FUSION)
uint8 detection_source

# 检测置信度 [0.0, 1.0]
float32 confidence

# 最后更新时间
builtin_interfaces/Time last_update_time
