import rclpy
from webots_ros2_core.webots_node import WebotsNode


class WebotsDriver(WebotsNode):
    def __init__(self, args):
        super().__init__('webots_driver', args=args)


def main(args=None):
    rclpy.init(args=args)
    webots_driver = WebotsDriver(args=args)
    rclpy.spin(webots_driver)
    webots_driver.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()