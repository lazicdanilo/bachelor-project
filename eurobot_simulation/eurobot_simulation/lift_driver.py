import rclpy
from std_msgs.msg import Float32


class LiftDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__lift_right = self.__robot.getDevice('lift_right')
        self.__lift_left = self.__robot.getDevice('lift_left')
        self.__lift_center = self.__robot.getDevice('lift_center')

        rclpy.init(args=None)
        self.__node = rclpy.create_node('lift_driver')
        self.__node.create_subscription(Float32, 'lift_right', self.__move_right_lift, 1)
        self.__node.create_subscription(Float32, 'lift_left', self.__move_left_lift, 1)
        self.__node.create_subscription(Float32, 'lift_center', self.__move_center_lift, 1)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

    def __move_right_lift(self, position):
        self.__lift_right.setPosition(position.data)

    def __move_left_lift(self, position):
        self.__lift_left.setPosition(position.data)

    def __move_center_lift(self, position):
        self.__lift_center.setPosition(position.data)
