#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class BallPositionDebugger(Node):
    def __init__(self):
        super().__init__('ball_debugger')

        # 订阅spawn脚本发布的球位置
        self.spawn_sub = self.create_subscription(
            Float32MultiArray,
            'spawned_ball_positions',
            self.spawn_callback,
            10)

        # 订阅视觉检测器发布的点云
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            'ball_positions',
            self.cloud_callback,
            10)

        self.get_logger().info('球位置调试器已启动')

    def spawn_callback(self, msg):
        self.get_logger().info(f'收到spawn脚本的球位置: {len(msg.data)//3} 个球')
        for i in range(0, len(msg.data), 3):
            x, y, z = msg.data[i:i+3]
            self.get_logger().info(f'  球{i//3+1}: ({x:.2f}, {y:.2f}, {z:.2f})')

    def cloud_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=['x', 'y', 'z']))
        self.get_logger().info(f'收到视觉检测的点云: {len(points)} 个点')
        for i, point in enumerate(points):
            self.get_logger().info(f'  点{i+1}: ({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f})')

def main():
    rclpy.init()
    node = BallPositionDebugger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()