import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool  # message for lifts and vacuum pumps
from geometry_msgs.msg import Twist     # message for diff drive
from sensor_msgs.msg import Range     # message for diatance sensors
from eurobot_fsm.navigation import Navigation
import asyncio
import threading

class EurobotFSM(Node):
    def __init__(self):
        super().__init__('eurobot_fsm')
        self.nav = Navigation(node=self)
        self.__step_counter = 0

        self.get_logger().info(f'Robot FSM init done')
        
        # self.timer = self.create_timer(0.1, self.timer_callback)

        self.__right_lift_publisher = self.create_publisher(Float32, 'lift_right', 10)
        self.__left_lift_publisher = self.create_publisher(Float32, 'lift_left', 10)
        self.__center_lift_publisher = self.create_publisher(Float32, 'lift_center', 10)

        self.__right_vacuum_pump_publisher = self.create_publisher(Bool, 'vacuum_pump_right', 10)
        self.__left_vacuum_pump_publisher = self.create_publisher(Bool, 'vacuum_pump_left', 10)
        self.__center_vacuum_pump_publisher = self.create_publisher(Bool, 'vacuum_pump_center', 10)

        # self.__diff_drive_velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.create_subscription(Range, 'robot/distance_sens_right', self.__check_distance_sensors, 10)
        self.create_subscription(Range, 'robot/distance_sens_left', self.__check_distance_sensors, 10)

        asyncio.run (self.main_task2())


    async def main_task2(self):
        while True:
            self.get_logger().info(f'<<<<<<<<< task tick >>>>>>>>>>')
            await asyncio.sleep(0.5)

    def timer_callback(self):
        self.__step_counter += 1
        # self.main_task(self.__step_counter)

        if self.__step_counter == 20:
            self.nav.go_forward(-.20)

        elif self.__step_counter == 30:
            self.nav.rotate(-185)

        elif self.__step_counter == 80:
            self.nav.go_forward(.46)

        elif self.__step_counter == 100:
            self.nav.rotate(185)

        elif self.__step_counter == 140:
            self.__lift_control('down')
            self.nav.go_forward(1.27)

        elif self.__step_counter == 180:
            self.__vacuum_pumps_control('on')

        elif self.__step_counter == 185:
            self.nav.go_forward(-1.22)

        elif self.__step_counter == 225:
            self.nav.rotate(175)

        elif self.__step_counter == 265:
            self.nav.go_forward(.36)

        elif self.__step_counter == 275:
            self.nav.rotate(-185)

        elif self.__step_counter == 300:
            self.__vacuum_pumps_control('on')

        # self.__vacuum_pumps_control('off')
        # self.nav.go_x(-.15)
        # self.nav.rotate(-110)
        # self.nav.go_x(.30)
        # self.nav.rotate(110)
        # self.nav.go_x(1.2)





    def main_task(self, tick):
        if self.__step_counter == 20:
            self.nav.go_forward(1.5)
            self.__lift_control('down')

        elif self.__step_counter == 70:
            self.__vacuum_pumps_control('on')

        elif self.__step_counter == 80:
            self.nav.go_forward(-1.55)

        elif self.__step_counter == 120:
            self.__vacuum_pumps_control('off')

        elif self.__step_counter == 125:
            self.nav.go_forward(-.15)

        elif self.__step_counter == 135:
            self.nav.rotate(-185)

        elif self.__step_counter == 170:
            self.nav.go_forward(.70)
        
        elif self.__step_counter == 190:
            self.nav.rotate(185)

        elif self.__step_counter == 220:
            self.nav.go_forward(1.2)

    def __lift_control(self, data):
        msg = Float32()
        if data == 'up':
            msg.data = 0.1
        elif data == 'down':
            msg.data = 0.0
        self.__right_lift_publisher.publish(msg)
        self.__left_lift_publisher.publish(msg)
        self.__center_lift_publisher.publish(msg)


    def __vacuum_pumps_control(self, data):
        msg = Bool()
        if data == 'on':
            msg.data = True
        elif data == 'off':
            msg.data = False
        self.__right_vacuum_pump_publisher.publish(msg)
        self.__left_vacuum_pump_publisher.publish(msg)
        self.__center_vacuum_pump_publisher.publish(msg)

    def __check_distance_sensors(self, data):
        if data.range < 0.02:
            self.get_logger().info(f'HIT')
            
        

def main(args=None):
    rclpy.init(args=args)

    eurobot_fsm = EurobotFSM()

    th = threading.Thread(target=rclpy.spin, args=(eurobot_fsm,))
    th.start()
    
    # rclpy.spin(eurobot_fsm)

    eurobot_fsm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()