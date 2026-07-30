#!/usr/bin/env python3
"""
雷达感知模块启动文件 — ROBOCON 2027

模式:
  单体 (默认):  LidarPerceptionNode 内部串联全部管线
  组件:         启动 6 个独立子节点, 通过话题串联
  测试:         启动模拟点云发布者 (无雷达硬件时用)

使用:
  ros2 launch br_perception lidar_perception.launch.py
  ros2 launch br_perception lidar_perception.launch.py test_mode:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    lidar_params = os.path.join(pkg_dir, 'config', 'lidar_params.yaml')
    field_geometry = os.path.join(pkg_dir, 'config', 'field_geometry.yaml')

    # ── Launch 参数 ──
    test_mode_la = DeclareLaunchArgument(
        'test_mode', default_value='false',
        choices=['true', 'false'],
        description='Enable simulated point cloud (no hardware needed)')

    rviz_la = DeclareLaunchArgument(
        'rviz', default_value='true',
        choices=['true', 'false'],
        description='Launch RViz2')

    test_mode = LaunchConfiguration('test_mode')
    rviz = LaunchConfiguration('rviz')

    # ═══════════════════════════════════════════════════════════════════════
    # 感知主节点 (单体全管线)
    # ═══════════════════════════════════════════════════════════════════════
    perception_node = Node(
        package='br_perception',
        executable='lidar_perception_node',
        name='lidar_perception_node',
        output='screen',
        parameters=[lidar_params, field_geometry],
    )

    # ═══════════════════════════════════════════════════════════════════════
    # 模拟点云发布者 (仅在 test_mode:=true 时启动)
    # ═══════════════════════════════════════════════════════════════════════
    test_publisher = Node(
        package='br_perception',
        executable='test_cloud_publisher',
        name='test_cloud_publisher',
        output='screen',
        condition=IfCondition(test_mode),
        parameters=[{
            'publish_rate': 10.0,
        }],
    )

    # ═══════════════════════════════════════════════════════════════════════
    # RViz2 (仅在 rviz:=true 时启动)
    # ═══════════════════════════════════════════════════════════════════════
    rviz_config = os.path.join(pkg_dir, 'config', 'lidar_perception.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
    )

    # ═══════════════════════════════════════════════════════════════════════
    # 静态 TF: livox_frame → map (让 RViz 有 TF 树, 否则黑屏)
    # ═══════════════════════════════════════════════════════════════════════
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_livox',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'livox_frame'],
    )

    # ═══════════════════════════════════════════════════════════════════════
    ld = LaunchDescription()
    ld.add_action(test_mode_la)
    ld.add_action(rviz_la)
    ld.add_action(static_tf)
    ld.add_action(perception_node)
    ld.add_action(test_publisher)
    ld.add_action(rviz_node)

    return ld
