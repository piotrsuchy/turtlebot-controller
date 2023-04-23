import rclpy
import tf_transformations
from math import sqrt, atan2
from random import random
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self, trajectory: list[tuple], v_const = 0.0, wgain = 1.0, distThresh1 = 0.8, distThresh2 = 0.10):
        super().__init__('dist_controller')
        self.v_const = v_const
        self.wgain = wgain
        # normal mode is when both v_const and wgain are published
        self.normal_mode = False
        # first threshold is when we start to slow down, second is when we have very slow speed
        self.distThresh1 = distThresh1
        self.distThresh2 = distThresh2
        self.target_x = None
        self.target_y = None
        # trajectory is a list of tuples of target positions
        self.trajectory = trajectory
        self.idx_tr = 0
        self.goal_reached = False
        # position
        self.x = 0
        self.y = 0
        self.z = 0
        # quaternions
        self.q_x = 0
        self.q_y = 0
        self.q_z = 0
        self.q_w = 0
        # publisher - publishing linear and angular velocity 
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # subsciber - updating position and quaternions
        self.subscriber = self.create_subscription(Odometry, 'odom', self.update_position, 10)
        self.set_target_position(self.trajectory[0][0], self.trajectory[0][1])
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def set_target_position(self, target_x, target_y):
        self.goal_reached = False
        self.normal_mode = False
        self.target_x = target_x
        self.target_y = target_y
        # print to console for debugging purposes
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
        # run calculate angle function in which we change self.v_const and self.wgain
        self.calculate_angle()


    # publisher callback to give linear velocity and angular velocity
    # calculated in calculate_velocity function
    def publisher_callback(self):
        if self.goal_reached is False:
            msg = Twist()
            msg.linear.x = self.v_const
            msg.angular.z = self.wgain
            # publish the linear velocity for x axis, and angular velocity for z:
            self.publisher.publish(msg)

    # in timer callback we send info to terminal
    def timer_callback(self):
        if self.goal_reached is False:
            # string_to_send = 'v_const = {}, v_ang = {}, normal_mode: {}'.format(self.v_const, self.wgain, self.normal_mode)
            # self.get_logger().info(string_to_send)
            diff = goal_angle - yaw
            self.get_logger().info(f"Angle difference in rad: {diff:.2f}")
            self.get_logger().info(f"Velocity: {self.v_const:.2f}")
        else:
            self.get_logger().info("Destination reached")

    # a function to fully stop the bot and change the normal_mode to false
    def full_stop(self):
        self.normal_mode = False
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
                if self.normal_mode is False: 
                    if abs(goal_angle - yaw) >= 0.001:
                        if goal_angle - yaw <= 0.0:
                            self.wgain = -(goal_angle - yaw)
                        # calculate angular velocity the bigger the difference, the higher the velocity:
                        self.wgain = goal_angle - yaw
                    else:
                        self.normal_mode = True
                        if self.normal_mode is True:
                            self.wgain = 0.0
                        # publish changes with just angle changed
                    self.publisher_callback()
                else:
                    # if self.normal mode is True we calculate distance 
                    # to goal and change v_const based on that
                    distance_to_goal = sqrt(abs(dx**2 + dy**2))
                    if distance_to_goal > self.distThresh1:
                        if self.v_const <= 0.8:
                            self.v_const += 0.01
                        else:
                            self.v_const = min(0.8, distance_to_goal)
                        self.wgain = goal_angle - yaw
                    elif distance_to_goal > self.distThresh2:
                        self.v_const = 0.1
                        self.wgain = goal_angle - yaw
                    else:
                        self.full_stop()
                        self.goal_reached = True
                    self.publisher_callback()
            else:
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
    controller = Controller(trajectory=[(3, 2), (1, 4), (-1, 5), (-3, 4), (-4, 2), (0, 0)])
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        