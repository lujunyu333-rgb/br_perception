# CameraBuildingSpot.msg — 相机检测到的建筑位绿色方格（图像空间结果）
#
# 与 BuildingSpot.msg（世界坐标系，融合级）不同，
# 本消息承载纯图像空间的检测原始结果，供调试、验证和融合模块使用。

# 相机 ID: 0 = 前向 (front), 1 = 后向 (rear)
uint8 camera_id

# 是否检测到建筑位
bool detected

# 检测到的四角像素坐标 (透视投影后的四边形，非严格矩形)
# 顺序: 左上、右上、右下、左下（按轮廓点排序）
float32 u0
float32 v0
float32 u1
float32 v1
float32 u2
float32 v2
float32 u3
float32 v3

# 中心像素坐标
float32 center_u
float32 center_v

# 检测置信度 [0.0, 1.0]
float32 confidence
