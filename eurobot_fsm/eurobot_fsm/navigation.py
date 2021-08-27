from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math


class Navigation:
    def __init__(self, node):
        
        self.__node = node

        self.__robot_x = 0
        self.__robot_y = 0
        self.__robot_x_desired = None
        self.__robot_x_err = 0

        self.flg = False
        self.flg2 = False
        
        self.__robot_angle = 0
        self.__prev_robot_angle = 0
        self.__robot_angle_sum = 0
        self.__no_of_turns = 0
        self.__desired_robot_angle = None
        self.__robot_robot_angle_error = 0
        
        self.distance_traveled = 0
        self.distance_traveled_desired = 0
        self.prev_x = 0
        self.prev_y = 0

        self.__diff_drive_velocity_publisher = self.__node.create_publisher(Twist, 'cmd_vel', 10)

        self.__node.create_timer(0.1, self.__regulation)

        self.__node.create_subscription(Odometry, '/odom', self.__get_odom_data, 1)

        self.__node.get_logger().info(f'NAVIGATION init done')

    def go_forward(self, forward_amount): 
        self.__node.get_logger().info(f'Go forward = {forward_amount}')   
        self.distance_traveled_desired = self.distance_traveled + forward_amount

    def go_x(self, x):
        self.__node.get_logger().info(f'Go forward x = {x}')
        self.__robot_x_desired =  self.__robot_x + x

        # if self.__robot_x > 0:
            # self.__robot_x_desired =  self.__robot_x - x
        # else:
            # self.__robot_x_desired =  self.__robot_x + x

        self.__node.get_logger().info(f'Dsired x = {self.__robot_x_desired}')

    # def go_angle(self, angle):
    #     self.__node.get_logger().info(f'Go angle angle = {angle}')
    #     self.__desired_robot_angle = angle
    
    def rotate(self, angle):
        self.__desired_robot_angle = self.__robot_angle_sum +  angle

    
    def __get_odom_data(self, data):
        self.__robot_x = data.pose.pose.position.x
        self.__robot_y = data.pose.pose.position.y

        r, p, self.__robot_angle = self.euler_from_quaternion(data.pose.pose.orientation.x,
                                                        data.pose.pose.orientation.y,
                                                        data.pose.pose.orientation.z,
                                                        data.pose.pose.orientation.w)

        if self.flg2 == False:
            self.distance_traveled = 0
            self.prev_x = self.__robot_x 
            self.prev_y = self.__robot_y 
            self.flg2 = True
        else:
            if self.distance_traveled_desired < self.distance_traveled:    # going backward
                self.distance_traveled -= math.sqrt((self.__robot_x - self.prev_x)**2 + (self.__robot_y - self.prev_y)**2)
            else:    # going forward
                self.distance_traveled += math.sqrt((self.__robot_x - self.prev_x)**2 + (self.__robot_y - self.prev_y)**2)
                
        self.__robot_angle = self.__robot_angle*57.296  # get angle in deg
        if self.__robot_angle < 0:                      # get angle in deg from 0 to 360
            self.__robot_angle = self.__robot_angle*(-1)
            self.__robot_angle = 360 - self.__robot_angle 
        

        if self.flg == False:
            self.__prev_robot_angle = self.__robot_angle
            self.flg = True

        if abs(self.__prev_robot_angle - self.__robot_angle) > 200:
            if self.__prev_robot_angle - self.__robot_angle < 180:
                self.__no_of_turns -= 1
            elif self.__prev_robot_angle - self.__robot_angle > 180:
                self.__no_of_turns += 1

        self.__robot_angle_sum = self.__robot_angle + self.__no_of_turns*360

        
        self.__node.get_logger().info(f'X = {self.__robot_x}, Y = {self.__robot_y}')
        self.__node.get_logger().info(f'distance traveled = {self.distance_traveled}')
        # self.__node.get_logger().info(f'Y = {self.__robot_y}')
        # self.__node.get_logger().info(f'Z = {data.pose.pose.position.z}')
        # self.__node.get_logger().info(f'prev ang = {self.__prev_robot_angle}')
        # self.__node.get_logger().info(f'ang = {self.__robot_angle}')
        # self.__node.get_logger().info(f'ang sum = {self.__robot_angle_sum}')

        self.__prev_robot_angle = self.__robot_angle
        self.prev_x = self.__robot_x
        self.prev_y = self.__robot_y

    def __regulation(self): 
        msg = Twist()

        # if self.__robot_x_desired != None:
        #     self.__robot_x_err = (self.__robot_x_desired - self.__robot_x)
        # else:
        #     self.__robot_x_err = 0
        msg.linear.x = float((self.distance_traveled_desired - self.distance_traveled) * 5)
        # msg.linear.x = float(0.1)
        
        if self.__desired_robot_angle != None:
            self.__robot_robot_angle_error = self.__desired_robot_angle - self.__robot_angle_sum 
        else:
            self.__robot_robot_angle_error = 0
        msg.angular.z = float(self.__robot_robot_angle_error * 0.02)
        # msg.angular.z = float(self.__robot_robot_angle_error)
        # msg.angular.z = float(.2)
        # self.__node.get_logger().info(f'err ang = {self.__robot_robot_angle_error}')
        
        self.__diff_drive_velocity_publisher.publish(msg)


    def euler_from_quaternion(self, x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
        
    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q