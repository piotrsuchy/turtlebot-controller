import rclpy
import tf_transformations
from math import sqrt, atan2
from random import random
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self, trajectory: list[tuple], v_const = 0.0, wgain = 1.0, distThresh1 = 0.5, distThresh2 = 0.10):
        super().__init__('dist_controller')
        self.v_const = v_const
        self.wgain = wgain
        self.normal_mode = False
        self.distThresh1 = distThresh1
        self.distThresh2 = distThresh2
        self.target_x = None
        self.target_y = None
        self.trajectory = trajectory
        self.idx_tr = 0
        self.goal_reached = False
        self.x = 0
        self.y = 0
        self.z = 0
        self.q_x = 0
        self.q_y = 0
        self.q_z = 0
        self.q_w = 0
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, 'odom', self.update_position, 10)
        self.set_target_position(self.trajectory[0][0], self.trajectory[0][1])
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def set_target_position(self, target_x, target_y):
        self.goal_reached = False
        self.normal_mode = False
        self.target_x = target_x
        self.target_y = target_y
        print("Traget position: {} and {}".format(self.target_x, self.target_y))

    # function for reading position and orientation in a subscriber
    def update_position(self, msg):
        posit = msg.pose.pose.position
        orient = msg.pose.pose.orientation 
        self.x = posit.x
        self.y = posit.y
        self.z = posit.z
        self.q_x = orient.x
        self.q_y = orient.y
        self.q_z = orient.z
        self.q_w = orient.w
        self.calculate_angle()


    # publisher callback to give linear velocity and angular velocity
    # calculated in calculate_velocity function
    def publisher_callback(self):
        msg = Twist()
        msg.linear.x = self.v_const
        msg.angular.z = self.wgain
        # publish the linear velocity for x axis, and angular velocity for z:
        self.publisher.publish(msg)

    def timer_callback(self):
        string_to_send = 'X: {0:.2f} Y:{1:.2f} Z: {2:.2f} --- ang_x: {3:.2f}, ang_y: {4:.2f}, ang_z: {5:.2f}, ang_w: {6:.2f}'.format(self.x, self.y, self.z, self.q_x, self.q_y, self.q_z, self.q_w)
        diff = str(goal_angle - yaw)
        self.get_logger().info(string_to_send)
        self.get_logger().info(diff)

    def full_stop(self):
        self.v_const = 0.0
        self.wgain = 0.0
        self.publisher_callback()

    def calculate_angle(self):
        if (self.target_x is None or self.target_y is None):
            print("The target destination has not been given yet!")
        else:
            global goal_angle
            global yaw
            if self.goal_reached is False:
                # calculate angle we need to have to reach the goal
                dx = self.target_x - self.x
                dy = self.target_y - self.y
                goal_angle = atan2(dy, dx)
                # calculate angles from quaternions, yaw is the third one (for z)
                angles = tf_transformations.euler_from_quaternion((self.q_x,self.q_y,
                                                                self.q_z,self.q_w))
                yaw = angles[2]
                if self.normal_mode is False and goal_angle - yaw >= 0.005:
                    # calculate angular velocity the bigger the difference, the higher the velocity:
                    self.wgain = goal_angle - yaw
                    # publish changes with just angle changed
                    self.publisher_callback()
                    if abs(goal_angle - yaw) >= 0.005: 
                        self.normal_mode = True
                    if self.normal is True:
                        self.wgain = 0.0
                else:
                    self.normal_mode = True
                    distance_to_goal = sqrt(abs(dx**2 + dy**2))
                    if distance_to_goal > self.distThresh1:
                        self.v_const = min(0.2, distance_to_goal)
                        self.wgain = goal_angle - yaw
                    elif distance_to_goal > self.distThresh2:
                        self.v_const = 0.05
                        self.wgain = goal_angle - yaw
                    else:
                        self.v_const = 0.0
                        self.wgain = 0.0
                        self.goal_reached = True
                    self.publisher_callback()
            else:
                print(self.goal_reached)
                self.full_stop()
                print(f"We arrived to the destination of index {self.idx_tr}")
                if self.idx_tr < len(self.trajectory)-1:
                    self.idx_tr += 1
                    self.set_target_position(self.trajectory[self.idx_tr][0], self.trajectory[self.idx_tr][1])
                else:
                    self.full_stop()
                    print("End of the road")

                


def main(args = None):
    
    rclpy.init(args=args)
    controller = Controller(trajectory=[(10, 10), (0, 0), (2, 2)])
    # controller.set_target_position(10, 10)
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        