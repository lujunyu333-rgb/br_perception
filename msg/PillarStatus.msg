# 柱子状态消息
# 描述场地中柱子的几何信息和状态（穆斯蒂卡柱或核心支柱）

# 柱子类型
# 0 = 穆斯蒂卡柱 (MUSTIKA_PILLAR)  — 穆斯蒂卡初始放置的柱子
# 1 = 核心支柱 (CORE_PILLAR)        — 场地核心支柱
uint8 type

# 柱子中心位置（世界坐标系，地面投影点）
geometry_msgs/Point position

# 柱子高度 [m]
float32 height

# 柱子直径 [m]
float32 diameter

# 柱子顶部是否检测到物体（穆斯蒂卡或方块）
bool has_object_on_top

# 检测置信度 [0.0, 1.0]
float32 confidence
