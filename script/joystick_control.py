#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_twist_message)
        
    def publish_twist_message(self):
        twist = Twist()
        twist.linear.x = 0.5  # Move forward
        twist.angular.z = 0.3  # Rotate

        self.publisher_.publish(twist)
        self.get_logger().info(f'Publishing Twist: linear={twist.linear.x}, angular={twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    twist_publisher = TwistPublisher()

    try:
        rclpy.spin(twist_publisher)
    except KeyboardInterrupt:
        print("Shutting down Twist publisher node")
    finally:
        twist_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
