#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RotateOnce(Node):
    def __init__(self):
        super().__init__('rotate_once_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = None
        
        # 360 degrees rotation at 1 rad/s
        self.duration = 6.5
        self.get_logger().info('Starting initial rotation...')

    def timer_callback(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            
        msg = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if (current_time - self.start_time) < self.duration:
            msg.angular.z = 1.0 # Rotate at 1 rad/s
        else:
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Rotation complete.')
            self.destroy_timer(self.timer)
            # Exit node since it's done
            raise SystemExit
            
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RotateOnce()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()