#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs import point_cloud2
import pclpy
from pclpy import pcl
import numpy as np
import threading
import time
from datetime import datetime

class LidarPreprocessor(Node):
    def __init__(self, options=None):
        super().__init__('lidar_preprocessor', options=options)

        # 声明参数
        self.declare_parameter('leaf_size', 0.1)
        self.declare_parameter('enable_outlier_filter', True)
        self.declare_parameter('outlier_mean_k', 50)
        self.declare_parameter('outlier_std_thresh', 1.0)
        self.declare_parameter('roi_min_x', -50.0)
        self.declare_parameter('roi_max_x', 50.0)
        self.declare_parameter('roi_min_y', -50.0)
        self.declare_parameter('roi_max_y', 50.0)
        self.declare_parameter('roi_min_z', -5.0)
        self.declare_parameter('roi_max_z', 5.0)
        self.declare_parameter('enable_self_filter', False)
        self.declare_parameter('self_radius_x', 1.0)
        self.declare_parameter('self_radius_y', 1.0)
        self.declare_parameter('self_radius_z', 1.0)

        # 读取参数
        self.leaf_size = self.get_parameter('leaf_size').value
        self.enable_outlier_filter = self.get_parameter('enable_outlier_filter').value
        self.outlier_mean_k = self.get_parameter('outlier_mean_k').value
        self.outlier_std_thresh = self.get_parameter('outlier_std_thresh').value
        self.roi_min_x = self.get_parameter('roi_min_x').value
        self.roi_max_x = self.get_parameter('roi_max_x').value
        self.roi_min_y = self.get_parameter('roi_min_y').value
        self.roi_max_y = self.get_parameter('roi_max_y').value
        self.roi_min_z = self.get_parameter('roi_min_z').value
        self.roi_max_z = self.get_parameter('roi_max_z').value
        self.enable_self_filter = self.get_parameter('enable_self_filter').value
        self.self_radius_x = self.get_parameter('self_radius_x').value
        self.self_radius_y = self.get_parameter('self_radius_y').value
        self.self_radius_z = self.get_parameter('self_radius_z').value

        # 动态参数回调
        self.add_on_set_parameters_callback(self.on_parameter_change)

        # 订阅原始点云（传感器数据QoS）
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.sub_raw = self.create_subscription(
            PointCloud2,
            '/livox/lidar/pointcloud',
            self.pointcloud_callback,
            qos_sensor
        )

        # 发布滤波后点云
        self.pub_filtered = self.create_publisher(
            PointCloud2,
            '/perception/lidar/filtered',
            qos_sensor
        )

        # 发布诊断信息
        self.pub_diagnostics = self.create_publisher(
            DiagnosticArray,
            '/perception/lidar/diagnostics',
            10
        )

        # 自过滤：订阅机器人位姿
        self.current_pose = None
        self.has_pose = False
        self.pose_mutex = threading.Lock()
        if self.enable_self_filter:
            self.sub_pose = self.create_subscription(
                PoseStamped,
                '/robot_pose',
                10,
                self.pose_callback
            )
            self.get_logger().info("Self-filter ENABLED")

        self.get_logger().info(
            f"LidarPreprocessor ready | leaf={self.leaf_size:.3f}m | "
            f"outlier={'on' if self.enable_outlier_filter else 'off'}(k={self.outlier_mean_k},σ={self.outlier_std_thresh:.1f}) | "
            f"roi_x=[{self.roi_min_x:.1f},{self.roi_max_x:.1f}] "
            f"roi_y=[{self.roi_min_y:.1f},{self.roi_max_y:.1f}] "
            f"roi_z=[{self.roi_min_z:.1f},{self.roi_max_z:.1f}]"
        )

    # ========================================================================
    # 点云回调
    # ========================================================================
    def pointcloud_callback(self, msg: PointCloud2):
        # 解包ROS消息到pcl点云
        cloud_in = self.ros_to_pcl(msg)
        input_points = cloud_in.size()
        if input_points == 0:
            self.get_logger().warning("Empty point cloud received", throttle_duration_sec=5.0)
            return

        # 滤波管线计时
        t0 = time.perf_counter()

        cloud_ds = self.downsample(cloud_in)
        t1 = time.perf_counter()

        cloud_clean = self.remove_outliers(cloud_ds)
        t2 = time.perf_counter()

        cloud_crop = self.crop_roi(cloud_clean)
        t3 = time.perf_counter()

        cloud_final = self.remove_self(cloud_crop)
        t4 = time.perf_counter()

        # 耗时统计（毫秒）
        total_ms = (t4 - t0) * 1000.0
        downsample_ms = (t1 - t0) * 1000.0
        outlier_ms = (t2 - t1) * 1000.0
        crop_ms = (t3 - t2) * 1000.0
        self_ms = (t4 - t3) * 1000.0

        # 发布滤波结果
        output_msg = self.pcl_to_ros(cloud_final, msg.header)
        self.pub_filtered.publish(output_msg)

        # 发布诊断
        self.publish_diagnostics(
            total_ms, downsample_ms, outlier_ms, crop_ms, self_ms,
            input_points, cloud_final.size()
        )

        # 延迟超标告警
        if total_ms > 20.0:
            self.get_logger().warning(
                f"Preprocessing slow: {total_ms:.1f} ms (input={input_points}, output={cloud_final.size()})",
                throttle_duration_sec=2.0
            )

    # ========================================================================
    # 滤波管线（各步骤独立）
    # ========================================================================
    def downsample(self, cloud):
        """近似体素降采样"""
        output = pcl.PointCloud.PointXYZ()
        voxel = pcl.filters.ApproximateVoxelGrid.PointXYZ()
        voxel.setInputCloud(cloud)
        voxel.setLeafSize(self.leaf_size, self.leaf_size, self.leaf_size)
        voxel.filter(output)
        return output

    def remove_outliers(self, cloud):
        """统计离群点去除"""
        if not self.enable_outlier_filter or cloud.size() < self.outlier_mean_k:
            return cloud
        output = pcl.PointCloud.PointXYZ()
        sor = pcl.filters.StatisticalOutlierRemoval.PointXYZ()
        sor.setInputCloud(cloud)
        sor.setMeanK(self.outlier_mean_k)
        sor.setStddevMulThresh(self.outlier_std_thresh)
        sor.setNegative(False)   # 保留内点
        sor.filter(output)
        return output

    def crop_roi(self, cloud):
        """ROI裁剪（XYZ联合）"""
        output = pcl.PointCloud.PointXYZ()
        box = pcl.filters.CropBox.PointXYZ()
        box.setInputCloud(cloud)
        box.setMin(
            self.roi_min_x, self.roi_min_y, self.roi_min_z, 1.0
        )
        box.setMax(
            self.roi_max_x, self.roi_max_y, self.roi_max_z, 1.0
        )
        box.setNegative(False)   # 保留内部
        box.filter(output)
        return output

    def remove_self(self, cloud):
        """自过滤（剔除机器人本体点云）"""
        if not self.enable_self_filter:
            return cloud

        with self.pose_mutex:
            if not self.has_pose:
                return cloud
            pose = self.current_pose

        output = pcl.PointCloud.PointXYZ()
        box = pcl.filters.CropBox.PointXYZ()
        box.setInputCloud(cloud)
        box.setMin(
            pose.position.x - self.self_radius_x,
            pose.position.y - self.self_radius_y,
            pose.position.z - self.self_radius_z,
            1.0
        )
        box.setMax(
            pose.position.x + self.self_radius_x,
            pose.position.y + self.self_radius_y,
            pose.position.z + self.self_radius_z,
            1.0
        )
        box.setNegative(True)   # 剔除内部
        box.filter(output)
        return output

    # ========================================================================
    # 诊断发布
    # ========================================================================
    def publish_diagnostics(self, total_ms, downsample_ms, outlier_ms,
                            crop_ms, self_ms, input_points, output_points):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'lidar_preprocessor'
        status.hardware_id = 'mid360'

        # 总体健康判定
        if total_ms < 15.0:
            status.level = DiagnosticStatus.OK
            status.message = 'OK'
        elif total_ms < 25.0:
            status.level = DiagnosticStatus.WARN
            status.message = 'SLOW'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = 'VERY SLOW'

        # 添加key-value
        def add_kv(key, value):
            kv = KeyValue()
            kv.key = key
            kv.value = str(value)
            status.values.append(kv)

        add_kv('total_ms', total_ms)
        add_kv('downsample_ms', downsample_ms)
        add_kv('outlier_ms', outlier_ms)
        add_kv('crop_ms', crop_ms)
        add_kv('self_ms', self_ms)
        add_kv('input_points', float(input_points))
        add_kv('output_points', float(output_points))
        if input_points > 0:
            reduction = 1.0 - output_points / input_points
            add_kv('reduction_ratio', reduction)
        else:
            add_kv('reduction_ratio', 0.0)
        add_kv('leaf_size', self.leaf_size)

        diag_array.status.append(status)
        self.pub_diagnostics.publish(diag_array)

    # ========================================================================
    # 动态参数更新
    # ========================================================================
    def on_parameter_change(self, params):
        result = SetParametersResult()
        result.successful = True
        result.reason = ''

        for param in params:
            name = param.name
            try:
                if name == 'leaf_size':
                    val = param.value
                    if val <= 0.0 or val > 1.0:
                        result.successful = False
                        result.reason = 'leaf_size must be in (0, 1.0]'
                        break
                    self.leaf_size = val
                elif name == 'enable_outlier_filter':
                    self.enable_outlier_filter = param.value
                elif name == 'outlier_mean_k':
                    val = param.value
                    if val < 3:
                        result.successful = False
                        result.reason = 'outlier_mean_k must be >= 3'
                        break
                    self.outlier_mean_k = val
                elif name == 'outlier_std_thresh':
                    val = param.value
                    if val <= 0.0:
                        result.successful = False
                        result.reason = 'outlier_std_thresh must be > 0'
                        break
                    self.outlier_std_thresh = val
                elif name == 'roi_min_x':
                    self.roi_min_x = param.value
                elif name == 'roi_max_x':
                    self.roi_max_x = param.value
                elif name == 'roi_min_y':
                    self.roi_min_y = param.value
                elif name == 'roi_max_y':
                    self.roi_max_y = param.value
                elif name == 'roi_min_z':
                    self.roi_min_z = param.value
                elif name == 'roi_max_z':
                    self.roi_max_z = param.value
                elif name == 'enable_self_filter':
                    self.enable_self_filter = param.value
                    if self.enable_self_filter and not hasattr(self, 'sub_pose'):
                        self.sub_pose = self.create_subscription(
                            PoseStamped,
                            '/robot_pose',
                            10,
                            self.pose_callback
                        )
                elif name == 'self_radius_x':
                    self.self_radius_x = param.value
                elif name == 'self_radius_y':
                    self.self_radius_y = param.value
                elif name == 'self_radius_z':
                    self.self_radius_z = param.value
                else:
                    self.get_logger().debug(f'Ignoring unknown param: {name}')
            except Exception as e:
                result.successful = False
                result.reason = f'Type mismatch for {name}: {str(e)}'
                break

        if result.successful:
            self.get_logger().info(
                f'Params updated | leaf={self.leaf_size:.3f} '
                f'outlier={"on" if self.enable_outlier_filter else "off"}(k={self.outlier_mean_k},σ={self.outlier_std_thresh:.1f}) '
                f'self={"on" if self.enable_self_filter else "off"}'
            )
        else:
            self.get_logger().warn(f'Param update rejected: {result.reason}')

        return result

    # ========================================================================
    # 位姿回调（自过滤用）
    # ========================================================================
    def pose_callback(self, msg: PoseStamped):
        with self.pose_mutex:
            self.current_pose = msg.pose
            self.has_pose = True

    # ========================================================================
    # ROS <-> PCL 转换辅助
    # ========================================================================
    @staticmethod
    def ros_to_pcl(ros_msg):
        """将sensor_msgs.PointCloud2转换为pcl.PointCloud.PointXYZ"""
        # 读取点云到numpy数组
        points = []
        for p in point_cloud2.read_points(ros_msg, field_names=('x', 'y', 'z'), skip_nans=True):
            points.append((p[0], p[1], p[2]))
        cloud = pcl.PointCloud.PointXYZ()
        if points:
            cloud.from_array(np.array(points, dtype=np.float32))
        return cloud

    @staticmethod
    def pcl_to_ros(cloud, header):
        """将pcl.PointCloud.PointXYZ转换为sensor_msgs.PointCloud2"""
        # 提取点云数据
        pts = cloud.xyz  # numpy array (N, 3)
        if pts.size == 0:
            # 空点云
            return point_cloud2.create_cloud_xyz32(header, np.zeros((0, 3), dtype=np.float32))
        # 创建消息
        return point_cloud2.create_cloud_xyz32(header, pts.astype(np.float32))


def main(args=None):
    rclpy.init(args=args)
    node = LidarPreprocessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()