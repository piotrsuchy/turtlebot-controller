import rclpy
from rclpy.node import Node
from random import random
from geometry_msgs.msg import Twist


class FirstPublisher(Node):

    def __init__(self):
        super().__init__('first_minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = float(random())
        msg.linear.y = float(random())
        msg.angular.z = float(random())
        string_to_send = f'vel_x: {msg.linear.x:.2f}, vel_y: {msg.linear.y:.2f}, ang.z {msg.angular.z:.2f}'
        self.publisher_.publish(msg)
        self.get_logger().info(string_to_send)
        

def main(args=None):
    rclpy.init(args=args)
    first_minimal_publisher = FirstPublisher()
    rclpy.spin(first_minimal_publisher)
    first_minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
