# 障碍物消息
# 描述单个障碍物的位置、尺寸、运动状态和分类信息

# 障碍物唯一标识符
uint32 id

# 障碍物中心位置（世界坐标系）
geometry_msgs/Point position

# 包围盒尺寸（长/宽/高）[m]
geometry_msgs/Vector3 bbox_size

# 速度向量 [m/s]
geometry_msgs/Vector3 velocity

# 障碍物类型标签
# 0 = 未知 (UNKNOWN)
# 1 = 敌方机器人 (ENEMY_ROBOT)
# 2 = 掉落方块 (DROPPED_BLOCK)
# 3 = 其他 (OTHER)
uint8 type

# 检测置信度 [0.0, 1.0]
float32 confidence

# 检测来源
# 0 = 雷达 (LIDAR)
# 1 = 相机 (CAMERA)
# 2 = 融合 (FUSION)
uint8 source
