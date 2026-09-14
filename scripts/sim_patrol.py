#!/usr/bin/env python3
"""
sim_patrol.py — 仿真 sensor_rig 巡游轨迹 (Gazebo + Point-LIO 验证用)

按"方框 + 回原点"路线发布 /cmd_vel, 给 Point-LIO 制造运动; 终点回到起点,
便于用起终点偏差评估漂移 (闭环测试, 对应 doc/odometry_tuning.md 待办项)。

用法:
  python3 scripts/sim_patrol.py                 # 默认 2m 方框, 走 2 圈
  python3 scripts/sim_patrol.py --side 3.0 --laps 1
"""

import argparse
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Patrol(Node):
    """直线往复 (沿 +x 前进 side 米 → 掉头 → 返回原点), 闭环便于评估漂移"""

    def __init__(self, side: float, laps: int, speed: float = 0.3, yaw_rate: float = 0.5):
        super().__init__('sim_patrol')
        self.pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.side_ = side
        self.laps_ = laps

        t_straight = side / speed
        t_turn = math.pi / yaw_rate          # 掉头 180°
        self.segments_ = []
        for _ in range(laps):
            self.segments_.append((speed, 0.0, t_straight))   # 直行
            self.segments_.append((0.0, yaw_rate, t_turn))    # 掉头
            self.segments_.append((speed, 0.0, t_straight))   # 返回
            self.segments_.append((0.0, yaw_rate, t_turn))    # 掉头回正
        self.segments_.append((0.0, 0.0, 5.0))                # 收尾停稳

        self.idx_ = 0
        self.t0_ = self.get_clock().now()
        self.timer_ = self.create_timer(0.05, self.tick)
        total = sum(s[2] for s in self.segments_)
        self.get_logger().info(
            f'巡游开始: {laps} 趟 × {side}m 往复, 预计 {total:.1f}s, 结束回到起点')

    def tick(self):
        if self.idx_ >= len(self.segments_):
            self.pub_.publish(Twist())      # 停
            self.get_logger().info('巡游结束 (已回到起点附近, 停稳)')
            rclpy.shutdown()
            return

        v, w, dur = self.segments_[self.idx_]
        elapsed = (self.get_clock().now() - self.t0_).nanoseconds / 1e9
        if elapsed >= dur:
            self.idx_ += 1
            self.t0_ = self.get_clock().now()
            return

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub_.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--side', type=float, default=2.0, help='方框边长 (m)')
    ap.add_argument('--laps', type=int, default=2, help='圈数')
    args = ap.parse_args()

    rclpy.init()
    node = Patrol(args.side, args.laps)
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
