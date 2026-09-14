cmake_minimum_required(VERSION 3.8)
project(br_perception)

# ── C++17 ──
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# ═══════════════════════════════════════════════════════════════════════════
# 依赖查找
# ═══════════════════════════════════════════════════════════════════════════
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search features)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 外部包: Livox 驱动 (桥接节点用它的 CustomMsg 头)。
# ⚠ 在 ~/ws_livox, 构建前必须 `source ~/ws_livox/install/setup.bash`, 否则 configure 直接报错。
find_package(livox_ros_driver2 REQUIRED)

# ═══════════════════════════════════════════════════════════════════════════
# 自定义消息生成
# ═══════════════════════════════════════════════════════════════════════════
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MustikaStatus.msg"
  "msg/PillarStatus.msg"
  "msg/Obstacle.msg"
  "msg/BuildingSpot.msg"
  "msg/PerceptionFrame.msg"
  "msg/CameraMustika.msg"
  "msg/CameraBuildingSpot.msg"
  "msg/CameraPillar.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces diagnostic_msgs
)

# 获取 C++ typesupport 链接目标（供组件库链接）
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# 公共 include 目录
include_directories(
  src/lidar/include
  src/camera/include
  src/vision/include
  src/fusion/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ═══════════════════════════════════════════════════════════════════════════
# 宏: 创建 ROS2 组件库
#   ARGN 自动捕获所有剩余参数 → 作为 ROS2 依赖包名列表传递给 ament
# ═══════════════════════════════════════════════════════════════════════════
macro(add_lidar_component target_name source_file plugin_class executable_name)
  add_library(${target_name} SHARED ${source_file})
  ament_target_dependencies(${target_name} ${ARGN})
  target_link_libraries(${target_name}
    ${PCL_LIBRARIES}
    ${cpp_typesupport_target})
  rclcpp_components_register_node(${target_name}
    PLUGIN "${plugin_class}"
    EXECUTABLE ${executable_name})
endmacro()

# ═══════════════════════════════════════════════════════════════════════════
# 1. LidarPreprocessor — 预处理
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(lidar_preprocessor_component
  src/lidar/lidar_preprocessor.cpp
  "br_perception::lidar::LidarPreprocessor"
  lidar_preprocessor
  rclcpp rclcpp_components sensor_msgs geometry_msgs diagnostic_msgs pcl_conversions)

# ═══════════════════════════════════════════════════════════════════════════
# 2. GroundSegmenter — 地面分割
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(ground_segmenter_component
  src/lidar/ground_segmenter.cpp
  "br_perception::lidar::GroundSegmenter"
  ground_segmenter
  rclcpp rclcpp_components sensor_msgs std_msgs diagnostic_msgs pcl_conversions)

# ═══════════════════════════════════════════════════════════════════════════
# 3. CylinderDetector — 圆柱检测
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(cylinder_detector_component
  src/lidar/cylinder_detector.cpp
  "br_perception::lidar::CylinderDetector"
  cylinder_detector
  rclcpp rclcpp_components sensor_msgs visualization_msgs diagnostic_msgs pcl_conversions)

# ═══════════════════════════════════════════════════════════════════════════
# 4. ClusterExtractor — 障碍物聚类
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(cluster_extractor_component
  src/lidar/cluster_extractor.cpp
  "br_perception::lidar::ClusterExtractor"
  cluster_extractor
  rclcpp rclcpp_components sensor_msgs visualization_msgs diagnostic_msgs pcl_conversions)

# ═══════════════════════════════════════════════════════════════════════════
# 5. BuildingSpotAnalyzer — 建筑位分析
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(building_spot_analyzer_component
  src/lidar/building_spot_analyzer.cpp
  "br_perception::lidar::BuildingSpotAnalyzer"
  building_spot_analyzer
  rclcpp rclcpp_components sensor_msgs visualization_msgs diagnostic_msgs pcl_conversions)

# ═══════════════════════════════════════════════════════════════════════════
# 6. LidarPerceptionNode — 主节点 (全管线编排)
#    额外链接 Eigen3::Eigen (非 ament 包, 单独处理)
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(lidar_perception_node_component
  src/lidar/lidar_perception_node.cpp
  "br_perception::lidar::LidarPerceptionNode"
  lidar_perception_node
  rclcpp rclcpp_components sensor_msgs visualization_msgs diagnostic_msgs std_msgs pcl_conversions)
target_link_libraries(lidar_perception_node_component Eigen3::Eigen)

# ═══════════════════════════════════════════════════════════════════════════
# 7. ImagePreprocessor — 双相机图像预处理
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(image_preprocessor_component
  src/camera/image_preprocessor.cpp
  "br_perception::camera::ImagePreprocessor"
  image_preprocessor
  rclcpp rclcpp_components sensor_msgs diagnostic_msgs cv_bridge)
target_link_libraries(image_preprocessor_component ${OpenCV_LIBRARIES})

# ═══════════════════════════════════════════════════════════════════════════
# 8. VisionUtils — 传统 CV 颜色工具库 (被 MustikaDetector 等链接)
# ═══════════════════════════════════════════════════════════════════════════
add_library(vision_utils SHARED src/vision/color_utils.cpp)
ament_target_dependencies(vision_utils
  rclcpp rclcpp_components sensor_msgs cv_bridge)
target_link_libraries(vision_utils ${OpenCV_LIBRARIES})

# ═══════════════════════════════════════════════════════════════════════════
# 9. MustikaDetector — 穆斯蒂卡相机检测
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(mustika_detector_component
  src/camera/mustika_detector.cpp
  "br_perception::camera::MustikaDetector"
  mustika_detector
  rclcpp rclcpp_components sensor_msgs diagnostic_msgs cv_bridge)
target_link_libraries(mustika_detector_component
  ${OpenCV_LIBRARIES}
  vision_utils
  ${cpp_typesupport_target})

# ═══════════════════════════════════════════════════════════════════════════
# 10. BuildingSpotDetector — 建筑位绿色方格检测
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(building_spot_detector_component
  src/camera/building_spot_detector.cpp
  "br_perception::camera::BuildingSpotDetector"
  building_spot_detector
  rclcpp rclcpp_components sensor_msgs diagnostic_msgs cv_bridge)
target_link_libraries(building_spot_detector_component
  ${OpenCV_LIBRARIES}
  vision_utils
  ${cpp_typesupport_target})

# ═══════════════════════════════════════════════════════════════════════════
# 11. PillarDetector — 柱子检测 (深绿穆斯蒂卡柱 + 棕色核心支柱)
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(pillar_detector_component
  src/camera/pillar_detector.cpp
  "br_perception::camera::PillarDetector"
  pillar_detector
  rclcpp rclcpp_components sensor_msgs diagnostic_msgs cv_bridge)
target_link_libraries(pillar_detector_component
  ${OpenCV_LIBRARIES}
  vision_utils
  ${cpp_typesupport_target})

# ═══════════════════════════════════════════════════════════════════════════
# 12. CoordinateTransformer — 坐标变换 / 里程计可靠性 (模块 17)
#     辅助库 (构造需 rclcpp::Node*), 供后续 fusion_node 与 test 链接
# ═══════════════════════════════════════════════════════════════════════════
add_library(coordinate_transformer SHARED
  src/fusion/coordinate_transformer.cpp)
ament_target_dependencies(coordinate_transformer
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs)

# ═══════════════════════════════════════════════════════════════════════════
# 13. Livox CustomMsg ↔ PointCloud2 桥接节点
#     仿真: Gazebo PointCloud2 → CustomMsg → Point-LIO (与实机同走 lidar_type=1)
#     实机: 驱动 CustomMsg → PointCloud2 → 感知管线 (§3)
# ═══════════════════════════════════════════════════════════════════════════
# livox_ros_driver2 是接口包: 显式链接其 C++ typesupport (与本仓库自有消息同做法)
if(TARGET livox_ros_driver2__rosidl_typesupport_cpp)
  set(livox_cpp_typesupport livox_ros_driver2__rosidl_typesupport_cpp)
else()
  message(WARNING "livox_ros_driver2 未导出 C++ typesupport 目标 — "
                  "桥接节点可能在链接期报 CustomMsg 符号未定义")
  set(livox_cpp_typesupport "")
endif()

add_executable(livox_custom_to_pcl2 src/utils/livox_custom_to_pcl2.cpp)
ament_target_dependencies(livox_custom_to_pcl2
  rclcpp sensor_msgs pcl_conversions livox_ros_driver2)
target_link_libraries(livox_custom_to_pcl2
  ${PCL_LIBRARIES}
  ${livox_cpp_typesupport})

add_executable(pointcloud2_to_livox_custom
  src/utils/pointcloud2_to_livox_custom.cpp)
ament_target_dependencies(pointcloud2_to_livox_custom
  rclcpp sensor_msgs pcl_conversions livox_ros_driver2)
target_link_libraries(pointcloud2_to_livox_custom
  ${PCL_LIBRARIES}
  ${livox_cpp_typesupport})

# ═══════════════════════════════════════════════════════════════════════════
# 测试工具: 模拟点云发布者 (无雷达时用于开发测试)
# ═══════════════════════════════════════════════════════════════════════════
add_executable(test_cloud_publisher
  test/test_cloud_publisher.cpp)
ament_target_dependencies(test_cloud_publisher
  rclcpp sensor_msgs pcl_conversions)
target_link_libraries(test_cloud_publisher ${PCL_LIBRARIES})

# ═══════════════════════════════════════════════════════════════════════════
# 单元测试 (§11) — colcon test / 或 ros2 run 直接跑
# ═══════════════════════════════════════════════════════════════════════════
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_coordinate_transformer
    test/test_coordinate_transformer.cpp)
  target_link_libraries(test_coordinate_transformer coordinate_transformer)
  ament_target_dependencies(test_coordinate_transformer
    rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)
endif()

# ═══════════════════════════════════════════════════════════════════════════
# 安装
# ═══════════════════════════════════════════════════════════════════════════
install(TARGETS test_cloud_publisher
  DESTINATION lib/${PROJECT_NAME})

# 桥接节点 — launch 以 package+executable 引用, 必须落在 lib/<pkg>
install(TARGETS
  livox_custom_to_pcl2
  pointcloud2_to_livox_custom
  DESTINATION lib/${PROJECT_NAME})

# rclcpp_components_register_node(... EXECUTABLE <名>) 生成的独立可执行。
# launch 里 executable='lidar_perception_node' / 'mustika_detector' 这类名字引用的就是
# 它们 → 必须在 install 空间里可被 ros2 run / ros2 launch 找到。
# 显式列出 (与 ROS 2 demos 同做法), 不依赖宏内部是否自带 install 规则。
install(TARGETS
  lidar_preprocessor
  ground_segmenter
  cylinder_detector
  cluster_extractor
  building_spot_analyzer
  lidar_perception_node
  image_preprocessor
  mustika_detector
  building_spot_detector
  pillar_detector
  DESTINATION lib/${PROJECT_NAME})

# 组件库 (.so), rclcpp_components_register_node 只注册不安装
install(TARGETS
  lidar_preprocessor_component
  ground_segmenter_component
  cylinder_detector_component
  cluster_extractor_component
  building_spot_analyzer_component
  lidar_perception_node_component
  image_preprocessor_component
  vision_utils
  mustika_detector_component
  building_spot_detector_component
  pillar_detector_component
  coordinate_transformer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY config launch msg
  DESTINATION share/${PROJECT_NAME})

# ── 仿真资产 (bringup_gazebo.launch.py 需要) ──
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/sim_patrol.py")
  install(FILES scripts/sim_patrol.py
    DESTINATION share/${PROJECT_NAME}/scripts)
else()
  message(WARNING "找不到 scripts/sim_patrol.py — bringup_gazebo 的自动巡游会起不来")
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/out/gazebo")
  install(DIRECTORY scripts/out/gazebo/
    DESTINATION share/${PROJECT_NAME}/gazebo)
else()
  message(WARNING
    "找不到 scripts/out/gazebo — 先跑 `python3 scripts/make_gazebo_world.py`, "
    "否则 bringup_gazebo.launch.py 找不到世界文件")
endif()

ament_package()
