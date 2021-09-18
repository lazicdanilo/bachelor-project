from math import fabs, trunc
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool  # message for lifts and vacuum pumps
from geometry_msgs.msg import Twist     # message for diff drive
from sensor_msgs.msg import Range     # message for distance sensors
from eurobot_fsm.navigation import Navigation
from eurobot_fsm.image_processing import ImageProcessing
from rclpy.parameter import Parameter

class EurobotFSM(Node):

    DETECT_OBSTACLE_DISTANCE = 0.15

    def __init__(self):

        # This part is for making sure we use simulation time.
        # The Webots simulator will dictate the clock and we will use it as reference
        parameter = Parameter
        parameter.name = 'use_sim_time'
        parameter.value = True
        
        super().__init__('eurobot_fsm', parameter_overrides=[parameter])
        self.get_logger().info(f'eurobot_fsm node started')
        
        # Used for strategies. Will be incremented every timer tick. Except if there is an obstacle
        self.__step_counter = 0      

        # Flag that will tell us if there is are obstacles on left or right sensors
        self.__obstacle_detected_right = None
        self.__obstacle_detected_left = None

        # Global obsticale flag. Will be raised if there is an obstacle on any of the sensors
        self.__obstacle_detected = False
        
        # Will hold puck colors that will be returned from ImageProcessing class
        self.puck_colors = dict()
        
        # Timer for main tick
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Create publishers for lifts
        self.__right_lift_publisher = self.create_publisher(Float32, 'lift_right', 10)
        self.__left_lift_publisher = self.create_publisher(Float32, 'lift_left', 10)
        self.__center_lift_publisher = self.create_publisher(Float32, 'lift_center', 10)

        # Create publishers for vacuum pumps
        self.__right_vacuum_pump_publisher = self.create_publisher(Bool, 'vacuum_pump_right', 10)
        self.__left_vacuum_pump_publisher = self.create_publisher(Bool, 'vacuum_pump_left', 10)
        self.__center_vacuum_pump_publisher = self.create_publisher(Bool, 'vacuum_pump_center', 10)

        # Subscribe to the distance sensors
        self.create_subscription(Range, 'robot/distance_sens_right', self.__check_distance_sensors_right, 10)
        self.create_subscription(Range, 'robot/distance_sens_left', self.__check_distance_sensors_left, 10)
        
        self.nav = Navigation(node=self)
        self.image_processing = ImageProcessing(node=self, show_image=False, save_image_path='/home/danilo/eurobot_output_image.jpg')
        

    def timer_callback(self):
        if not self.__obstacle_detected:
            self.__step_counter += 1
        
        self.get_logger().info(f'Tick {self.__step_counter}')

        # Activate one of the strategies
        self.strategy_one(self.__step_counter)
        # self.strategy_two(self.__step_counter)

        # Check for obstacles
        self.__obstacle_check()
    
    def strategy_one(self, tick):
        if tick == 2:
            self.image_processing.start_image_processing()

        if tick == 10:  # Gives time to get the image and process it 
            self.puck_colors = self.image_processing.get_puck_colors()
            self.nav.set_speed(0.9)
            self.nav.go_forward(1.48)
            self.__lift_control('down')

        elif tick == 25:
            self.__vacuum_pumps_control('on')
        
        elif tick == 35: # go on blue 
            self.__lift_control('up')
            self.nav.go_forward(-0.95)
        
        elif tick == 45: # go on green. We have reached BLUE
            self.__lift_control('down')
            if not self.puck_colors['error']:
                for place in self.puck_colors:
                    if self.puck_colors[place] == 'blue':
                        self.__vacuum_pumps_control('off', place)
            self.nav.go_forward(-.30)
        
        elif tick == 55: # go on red, We have reached GREEN
            if not self.puck_colors['error']:
                for place in self.puck_colors:
                    if self.puck_colors[place] == 'green':
                        self.__vacuum_pumps_control('off', place)
            self.nav.go_forward(-.35)

        elif tick == 65:   # We are on RED. Dump everything left here  
            self.__vacuum_pumps_control('off')
            self.nav.go_forward(-.15)

        elif tick == 75:
            self.nav.rotate(-90)

        elif tick == 85:                
            self.nav.go_forward(.50)    # 53 if we stop it once | todo: reslove this 
        
        elif tick == 95:
            self.nav.rotate(90)

        elif tick == 110:
            self.__lift_control('down')
            self.nav.go_forward(1.18)

        elif tick == 125:
             self.__vacuum_pumps_control('on')
        
        elif tick == 130:
            self.__lift_control('up')

        elif tick == 135:
            self.nav.go_forward(-.98)

        elif tick == 155:
             self.nav.rotate(90)
             self.__lift_control('down')

        elif tick == 170:
             self.nav.go_forward(.25)
             self.__vacuum_pumps_control('off')


    def strategy_two(self, tick):
        if tick == 5:
            self.__lift_control('down')
            self.nav.go_forward(-.20)
        
        if tick == 20:
            self.__lift_control('down')
            self.nav.rotate(-90)

        if tick == 35:
            self.nav.go_forward(1.5)

        if tick == 55:
            self.nav.rotate(90)

        if tick == 70:
            self.nav.go_forward(1.07)
        
        if tick == 90:
            self.nav.rotate(-90)
            
        if tick == 105:
            self.nav.go_forward(.38)

        if tick == 115:
            self.nav.rotate(90)

        if tick == 130:
            self.nav.set_speed(0.5)
            self.nav.go_forward(.151)   # .148
        
        if tick == 150:
            self.__vacuum_pumps_control('on')
            self.nav.go_forward(-.151)
        
        if tick == 160:
            self.nav.set_speed(0.8)
            self.__lift_control('up')
            self.nav.rotate(130)

        if tick == 200:
            self.nav.go_forward(1.25)

        if tick == 215:
            self.nav.rotate(-40)

        if tick == 225:
            self.nav.go_forward(.4)

        if tick == 245:
            self.__vacuum_pumps_control('off')

        if tick == 255:
            self.nav.go_forward(.3)

    def __lift_control(self, data):
        msg = Float32()
        if data == 'up':
            msg.data = 0.1
        elif data == 'down':
            msg.data = 0.0
        self.__right_lift_publisher.publish(msg)
        self.__left_lift_publisher.publish(msg)
        self.__center_lift_publisher.publish(msg)


    def __vacuum_pumps_control(self, data, vacuum_pump='all'):
        msg = Bool()
        if data == 'on':
            msg.data = True
        elif data == 'off':
            msg.data = False
        if vacuum_pump == 'all':
            self.__right_vacuum_pump_publisher.publish(msg)
            self.__left_vacuum_pump_publisher.publish(msg)
            self.__center_vacuum_pump_publisher.publish(msg)

        elif vacuum_pump == 'right':
            self.__right_vacuum_pump_publisher.publish(msg)

        elif vacuum_pump == 'center':
            self.__center_vacuum_pump_publisher.publish(msg)

        elif vacuum_pump == 'left':
            self.__left_vacuum_pump_publisher.publish(msg)


    def __check_distance_sensors_right(self, data):
        # Check if the distance sensor data is lower than DETECT_OBSTACLE_DISTANCE
        if not self.__obstacle_detected_right and data.range < self.DETECT_OBSTACLE_DISTANCE:
            self.__obstacle_detected_right = data.range
        elif self.__obstacle_detected_right and data.range >= self.DETECT_OBSTACLE_DISTANCE:
            self.__obstacle_detected_right = None

    def __check_distance_sensors_left(self, data):
        # Check if the distance sensor data is lower than DETECT_OBSTACLE_DISTANCE
        if not self.__obstacle_detected_left and data.range < self.DETECT_OBSTACLE_DISTANCE:
            self.__obstacle_detected_left = data.range
        elif self.__obstacle_detected_left and data.range >= self.DETECT_OBSTACLE_DISTANCE:
            self.__obstacle_detected_left = None

    def __obstacle_check(self):
        # Check if there there are obsticales detected and notify the Navigation class
        if not self.__obstacle_detected and (self.__obstacle_detected_left is not None or self.__obstacle_detected_right is not None):
            self.__obstacle_detected = True
            
            if self.__obstacle_detected_left is None and self.__obstacle_detected_right is not None:
                self.nav.obstacle(self.__obstacle_detected_right)
            elif self.__obstacle_detected_right is None and self.__obstacle_detected_left is not None:
                self.nav.obstacle(self.__obstacle_detected_left)

            if self.__obstacle_detected_left is not None and self.__obstacle_detected_right is not None:
                if self.__obstacle_detected_left <= self.__obstacle_detected_right:
                    self.nav.obstacle(self.__obstacle_detected_left)
                else:
                    self.nav.obstacle(self.__obstacle_detected_right)

            self.get_logger().info(f'Obstacle detected')

        elif self.__obstacle_detected and self.__obstacle_detected_left is None and self.__obstacle_detected_right is None:
            self.__obstacle_detected = False
            self.nav.obstacle(None)
            self.get_logger().info(f'No more obstacle')

def main(args=None):
    rclpy.init(args=args)

    eurobot_fsm = EurobotFSM()

    executor = rclpy.get_global_executor()
    executor.add_node(eurobot_fsm)

    rclpy.spin(eurobot_fsm, executor)

    eurobot_fsm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()