# CameraPillar.msg — 相机检测到的柱子（图像空间结果）
#
# 穆斯蒂卡柱：深绿色 PVC 管，高 500mm，直径 270mm
# 核心支柱：棕色，高 800mm，直径 270mm

# 相机 ID: 0 = 前向 (front), 1 = 后向 (rear)
uint8 camera_id

# 是否检测到柱子
bool detected

# 柱子类型: 0 = 穆斯蒂卡柱 (MUSTIKA_PILLAR), 1 = 核心支柱 (CORE_PILLAR), 2 = 未知 (UNKNOWN)
uint8 pillar_type

# 包围盒中心像素坐标
float32 u
float32 v

# 包围盒宽高 (pixels)
float32 width
float32 height

# 检测置信度 [0.0, 1.0]
float32 confidence

# 柱顶是否有物体 (对雷达融合有帮助)
bool has_top_object
