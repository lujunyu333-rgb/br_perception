# CameraMustika.msg — 相机检测到的穆斯蒂卡（金色球体）图像空间结果
#
# 与 MustikaStatus.msg（世界坐标系，融合级）不同，
# 本消息承载纯图像空间的检测原始结果，供调试、验证和融合模块使用。

# 相机 ID: 0 = 前向 (front), 1 = 后向 (rear)
uint8 camera_id

# 球心在图像中的像素坐标
float32 u
float32 v

# 球的像素半径
float32 radius

# 检测置信度 [0.0, 1.0]
float32 confidence

# 穆斯蒂卡是否在视野中
bool detected
