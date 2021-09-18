from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math

class Navigation:
    ANGULAR_KP = 0.12
    ANGULAR_LIMIT = 20

    LINEAR_KP = 3.2
    LINEAR_LIMIT = 0.4
    
    def __init__(self, node):
        
        # Parent node
        self.__node = node

        # Robot x and y coordinates
        self.__robot_x = 0
        self.__robot_y = 0

        # Distance to the obstacle, None if there is no obstacle
        self.__obstacle_distance = None
        
        # Previous robot angle, used for zero deg crossing 
        self.__robot_angle_prev = None
        
        # Current robot angle in deg
        self.__robot_angle = 0
        
        # Number of turns. Every 360 deg we have +1 turn and every -360 we have -1 turn
        self.__no_of_turns = 0

        # Desired angle, this will be used along with __robot_angle to calculate the angular error for regulation 
        self.__desired_robot_angle = None

        # Will hold sum of distance traveled. Will increment when we are going forward and decrement when we are going backward
        self.__distance_traveled = 0  

        # This will be desired position we want to reach. With this and __distance_traveled we will calculate linear error for regulation
        self.__distance_traveled_desired = 0
        
        # Previous robot x and y coordinates, used to calculate the distance traveled in one step 
        self.__robot_x_prev = None
        self.__robot_y_prev = None

        # Desired speed
        self.__speed_coefficient = 1.0    # Between 0.0 and 1.0

        # This will get us data from odometry of robot 
        self.__node.create_subscription(Odometry, '/odom', self.__get_odom_data, 1)

        # This will tell the robot how to move
        self.__diff_drive_velocity_publisher = self.__node.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer for main regulation
        self.__node.create_timer(0.1, self.__regulation)

        # Prints data to terminal
        self.__node.get_logger().info(f'NAVIGATION init done')
    
    def set_speed(self, new_speed):
        assert (new_speed >= 0.0 and new_speed <= 1.0), "The speed needs to be between 0.0 and 1.0"
        self.__speed_coefficient = new_speed 

    def go_forward(self, forward_amount):
        self.__node.get_logger().info(f'Go forward = {forward_amount} [m]')
        self.__distance_traveled_desired = self.__distance_traveled + forward_amount
    
    def obstacle(self, obstacle_distance):
        self.__obstacle_distance = obstacle_distance

    def rotate(self, angle):
        self.__node.get_logger().info(f'Go angle = {angle} [deg]')
        angle = angle * 1.08  # Needs to be here because of the error in wheel radius 
        self.__desired_robot_angle = self.__robot_angle + angle
    
    # This will be called every time we have new data from odometry
    def __get_odom_data(self, data):
        self.__robot_x = data.pose.pose.position.x  # Get current x cordinate 
        self.__robot_y = data.pose.pose.position.y  # Get current y cordinate 

        # Get angle in rad from quaternions
        r, p, robot_angle = self.euler_from_quaternion(data.pose.pose.orientation.x,
                                                        data.pose.pose.orientation.y,
                                                        data.pose.pose.orientation.z,
                                                        data.pose.pose.orientation.w)

        # Check if this is the first callback of this function and initialize the variables
        if self.__robot_x_prev is None and self.__robot_y_prev is None: 
            self.__distance_traveled = 0
        else:
            # Calculate the difference in coordinates from the last time we were in this function 
            dx = self.__robot_x - self.__robot_x_prev
            dy = self.__robot_y - self.__robot_y_prev
            
            # Check if we are going backward
            if self.__distance_traveled_desired < self.__distance_traveled:
                # Add to sum of distance traveled the distance traveled in this step using Pythagorean theorem 
                self.__distance_traveled -= math.sqrt(dx**2 + dy**2)    
            # Else we are going forward
            else:    
                # Add to sum of distance traveled the distance traveled in this step using Pythagorean theorem
                self.__distance_traveled += math.sqrt(dx**2 + dy**2)

        # Save the values for next calculation
        self.__robot_x_prev = self.__robot_x
        self.__robot_y_prev = self.__robot_y

        robot_angle = robot_angle * 57.296    # Get robot angle in deg (180/π) = 57.296
        
        # If the angle is negative (CW) (Mathematical minus)
        if robot_angle < 0:
            # Get it in degres from 0 to 360
            robot_angle = robot_angle * (-1)
            robot_angle = 360 - robot_angle 
        
        # Check if this is the first callback of this function and initialize the variables
        if self.__robot_angle_prev is None:
            self.__robot_angle_prev = robot_angle

        # Check if the angle has changed more than 200 deg in the last step
        # This should not be possible, this will indicate that we have a zero deg crossing 
        if abs(self.__robot_angle_prev - robot_angle) > 200:
            # If the zero cross is from left to right (CW) decrement the number of turns 
            if self.__robot_angle_prev - robot_angle < 180:
                self.__no_of_turns -= 1

            # If the zero cross is from right to left (CCW) increment the number of turns 
            elif self.__robot_angle_prev - robot_angle > 180:
                self.__no_of_turns += 1

        # Save the values for next calculation
        self.__robot_angle_prev = robot_angle

        # The final robot angle is sum of current robot_angle for this step and number of turns * 360 
        self.__robot_angle = robot_angle + self.__no_of_turns*360


    def __regulation(self): 
        msg = Twist()   # Initialize the message variable for ROS2 communication

        # Check if there is an obstacle detected
        if self.__obstacle_distance is None:
    
            # ******************************* Linear controller *******************************
            
            # Calculate the linear error
            linear_error = self.__distance_traveled_desired - self.__distance_traveled
            
            # Change the linear limit according to desired __speed_coefficient
            linear_limit = self.LINEAR_LIMIT * self.__speed_coefficient

            # Check if the linear_error is above linear_limit and cap it if it is
            if abs(linear_error) >= linear_limit:
                if linear_error < 0:
                    linear_error = linear_limit * (-1)
                else:
                    linear_error = linear_limit

            # Calculate Proportional component in for linear P regulation
            linear_p = linear_error * self.LINEAR_KP

            self.__node.get_logger().info(f'linear P = {linear_p}')
            
            # Finally put the calculated linear_p in message that will be sent to the Webots node 
            msg.linear.x = float(linear_p)

            # ******************************* Angular controller ******************************

            # Check if we have changed the desired angle. If we have calculate the angular_error. If not the error is 0
            if self.__desired_robot_angle is None:
                angular_error = 0
            else:
                angular_error = self.__desired_robot_angle - self.__robot_angle 
            
            # Change the angular limit according to desired __speed_coefficient
            angular_limit = self.ANGULAR_LIMIT * self.__speed_coefficient

            # Check if the angular_error is above angular_limit and cap it if it is
            if abs(angular_error) >= angular_limit:
                if angular_error < 0:
                    angular_error = angular_limit * (-1)
                else:
                    angular_error = angular_limit

            # Calculate Proportional component in for angular P regulation
            angular_p = angular_error * self.ANGULAR_KP

            # Finally put the calculated angular_p in message that will be sent to the Webots node 
            msg.angular.z = float(angular_p)

        # If there is an obstacle stop the robot 
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        # Publish the message with data from webots diff driver node
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