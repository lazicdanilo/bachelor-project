import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool  # message for lifts and vacuum pumps
from geometry_msgs.msg import Twist     # message for dif drive

class EurobotFSM(Node):
    def __init__(self):
        super().__init__('eurobot_fsm')
        self.__step_counter = 0

        # This can't be here as it's initialized in lift_driver.py. 
        # rclpy.init(args=None)

        # Unfortunately, we cannot get an instance of the parent ROS node.
        # However, we can create a new one.
        # self.__node = rclpy.node.Node('robot_fsm')
        self.get_logger().info(f'Robot FSM init done')
        
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.__right_lift_publisher = self.create_publisher(Float32, 'lift_right', 1)
        self.__left_lift_publisher = self.create_publisher(Float32, 'lift_left', 1)
        self.__center_lift_publisher = self.create_publisher(Float32, 'lift_center', 1)

        self.__right_vacuum_pump_publisher = self.create_publisher(Bool, 'vacuum_pump_right', 1)
        self.__left_vacuum_pump_publisher = self.create_publisher(Bool, 'vacuum_pump_left', 1)
        self.__center_vacuum_pump_publisher = self.create_publisher(Bool, 'vacuum_pump_center', 1)

        self.__diff_drive_velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        

    def timer_callback(self):
        self.__step_counter += 1
        self.get_logger().info(f'FSM step = {self.__step_counter}')   
        # rclpy.spin_once(self.__node, timeout_sec=0)

        if self.__step_counter > 15 and self.__step_counter < 25:
            msg = Twist()
            self.get_logger().info(f'GOOOOO') 
            msg.linear.x = 0.1
            self.__diff_drive_velocity_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    eurobot_fsm = EurobotFSM()

    rclpy.spin(eurobot_fsm)

    eurobot_fsm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()