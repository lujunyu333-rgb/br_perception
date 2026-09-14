#!/usr/bin/env python3
"""
Point-LIO 建图启动 — 实机 (Livox Mid-360 + Point-LIO ROS2 移植版)

链路:
  livox_ros_driver2 ──/livox/lidar (CustomMsg)──┬─→ point_lio (里程计 + 建图)
                                                └─→ livox_custom_to_pcl2
                                                      → /livox/lidar/pointcloud (感知管线)

建图与存图:
  · 实时看: rviz 里 /cloud_registered (当前帧配准到世界系, Decay Time 累积)
  · 存 PCD: lio_ws 的 mid360.yaml 里 `pcd_save_en` 已打开 → **Ctrl-C 退出时**把整段
            点云写成一个文件:
              ~/桌面/scans.pcd
            (底层是编译期写死的 ROOT_DIR/PCD/, 已把该目录软链到桌面;
             本机磁盘紧张, 建图前先确认空间)
  · 注意: /Laser_map 话题只在初始化时发一次 (这个 fork 不再持续发布), 看实时地图请用
    /cloud_registered

前置:
  source /opt/ros/humble/setup.bash
  source ~/ws_livox/install/setup.bash          # 驱动
  source ~/lio_ws/install/setup.bash            # Point-LIO
  source ~/br_perception/install/setup.bash
  网卡: sudo ip addr add 192.168.1.5/24 dev ens37 && sudo ip link set ens37 up

用法:
  ros2 launch br_perception pointlio_mapping.launch.py
  ros2 launch br_perception pointlio_mapping.launch.py rviz:=false          # 无图形界面
  ros2 launch br_perception pointlio_mapping.launch.py pipeline:=true       # 同时给感知管线供点云
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_launch(share_dir, launch_file):
    """在 launch/ 与 launch_ROS2/ (livox_ros_driver2 用后者) 下找 launch 文件"""
    for sub in ('launch', 'launch_ROS2', ''):
        candidate = os.path.join(share_dir, sub, launch_file) if sub \
            else os.path.join(share_dir, launch_file)
        if os.path.exists(candidate):
            return candidate
    raise RuntimeError(
        f"在 {share_dir} 下找不到 {launch_file} (已试 launch/ 与 launch_ROS2/)")


def _include(pkg_name, launch_file, launch_arguments=None, source_hint="", condition=None):
    try:
        share = get_package_share_directory(pkg_name)
    except Exception as exc:  # PackageNotFoundError
        raise RuntimeError(
            f"找不到包 '{pkg_name}' — 请先 source 对应工作区。{source_hint}"
        ) from exc
    kwargs = {'condition': condition} if condition is not None else {}
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_resolve_launch(share, launch_file)),
        launch_arguments=(launch_arguments or {}).items(),
        **kwargs,
    )


def generate_launch_description():
    share = get_package_share_directory('br_perception')
    # Point-LIO 自带的建图视图 (fixed frame = camera_init, 含 /cloud_registered + /path)
    lio_rviz = os.path.join(get_package_share_directory('point_lio'),
                            'rviz_cfg', 'loam_livox.rviz')

    rviz_arg = DeclareLaunchArgument(
        'perception_rviz', default_value='true', choices=['true', 'false'],
        description='启动 RViz2 (Point-LIO 建图视图)')
    pipeline_arg = DeclareLaunchArgument(
        'pipeline', default_value='false', choices=['true', 'false'],
        description='桥接出 PointCloud2 供感知管线用 (纯建图不需要)')
    rviz = LaunchConfiguration('perception_rviz')
    pipeline = LaunchConfiguration('pipeline')

    # 1. 雷达驱动 (CustomMsg — Point-LIO 的 lidar_type:1 要求)
    livox_driver = _include(
        'livox_ros_driver2', 'msg_MID360_launch.py',
        source_hint='雷达驱动在 ~/ws_livox, 见 ~/livox_lab_up.sh')

    # 2. Point-LIO: 里程计 + 建图 (PCD 保存在 lio_ws 的 mid360.yaml 里控制)
    point_lio = _include(
        'point_lio', 'mapping_mid360.launch.py',
        launch_arguments={'rviz': 'false'},   # 用自己的建图视图, 不起它自带的
        source_hint='Point-LIO 在 ~/lio_ws (9000_point_lio_ros2_Mid-360)')

    # 3. 可选: CustomMsg → PointCloud2 (给感知管线)
    custom_to_pcl2 = Node(
        package='br_perception', executable='livox_custom_to_pcl2',
        name='livox_custom_to_pcl2', output='screen',
        parameters=[{'input_topic': '/livox/lidar',
                     'output_topic': '/livox/lidar/pointcloud',
                     'frame_id': 'livox_frame'}],
        condition=IfCondition(pipeline))

    # 4. 静态外参 (⚠ 占位值, 装车后实测回填)
    static_tf_livox = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_base_to_livox',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.25',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'])

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', lio_rviz], condition=IfCondition(rviz))

    return LaunchDescription([
        rviz_arg, pipeline_arg,
        LogInfo(msg="[br_perception] Point-LIO 建图: 走一圈后 Ctrl-C 即落盘 PCD "
                    "(~/lio_ws/src/9000_point_lio_ros2_Mid-360/PCD/)"),
        static_tf_livox,
        livox_driver,
        point_lio,
        custom_to_pcl2,
        rviz_node,
    ])
