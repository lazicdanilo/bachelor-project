import rclpy
from std_msgs.msg import Bool
from webots_ros2_driver_webots.controller import Node

class VacuumPumpDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__vacuum_pump_right = self.__robot.getDevice('vacuum_connector_right')
        self.__vacuum_pump_left = self.__robot.getDevice('vacuum_connector_left')
        self.__vacuum_pump_center = self.__robot.getDevice('vacuum_connector_center')

        # This can't be here as it's initialized in lift_driver.py. 
        # rclpy.init(args=None)

        self.__vacuum_pump_right.enablePresence(1)
        self.__vacuum_pump_left.enablePresence(1)
        self.__vacuum_pump_center.enablePresence(1)

        # Unfortunately, we cannot get an instance of the parent ROS node.
        # However, we can create a new one.
        self.__node = rclpy.node.Node('vacuum_pump_driver')        
        self.__node.create_subscription(Bool, 'vacuum_pump_right', self.__control_right_vacuum_pump, 1)
        self.__node.create_subscription(Bool, 'vacuum_pump_left', self.__control_left_vacuum_pump, 1)
        self.__node.create_subscription(Bool, 'vacuum_pump_center', self.__control_center_vacuum_pump, 1)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

    def __control_right_vacuum_pump(self, state):
        if state.data == True:
            self.__node.get_logger().info(f'Turning on right vacuum pump')
            self.__vacuum_pump_right.lock()
        else:
            self.__node.get_logger().info(f'Turning off right vacuum pump')
            self.__vacuum_pump_right.unlock()

    def __control_left_vacuum_pump(self, state):
        if state.data == True:
            self.__node.get_logger().info(f'Turning on left vacuum pump')
            self.__vacuum_pump_left.lock()
        else:
            self.__node.get_logger().info(f'Turning off left vacuum pump')
            self.__vacuum_pump_left.unlock()

    def __control_center_vacuum_pump(self, state):
        if state.data == True:
            self.__node.get_logger().info(f'Turning on center vacuum pump')
            self.__vacuum_pump_center.lock()
        else:
            self.__node.get_logger().info(f'Turning off center vacuum pump')
            self.__vacuum_pump_center.unlock()
