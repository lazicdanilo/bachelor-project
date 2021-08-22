import rclpy
from geometry_msgs.msg import Point


class LiftDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__lift_right = self.__robot.getMotor('lift_right')
        self.__lift_left = self.__robot.getMotor('lift_left')
        self.__lift_center = self.__robot.getMotor('lift_center')

        rclpy.init(args=None)
        self.__node = rclpy.create_node('lift_driver')
        self.__node.create_subscription(Point, 'lift_right', self.__move_right_lift, 1)
        self.__node.create_subscription(Point, 'lift_left', self.__move_left_lift, 1)
        self.__node.create_subscription(Point, 'lift_center', self.__move_center_lift, 1)

    def step(self):
        pass

    def __move_right_lift(self, position):
        self.__lift_right.setPosition(position.z)

    def __move_left_lift(self, position):
        self.__lift_left.setPosition(position.z)

    def __move_center_lift(self, position):
        self.__lift_center.setPosition(position.z)
