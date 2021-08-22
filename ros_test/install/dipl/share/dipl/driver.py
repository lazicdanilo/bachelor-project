import rclpy
from webots_ros2_core.webots_node import WebotsNode
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
# from geometry_msgs.msg import Twist
from webots_ros2_core.webots_differential_drive_node import WebotsDifferentialDriveNode
# from webots_ros2_core.joint_state_publisher import JointState
# from trajectory_msgs.msg import JointTrajectoryPoint

DEFAULT_WHEEL_DISTANCE = 0.26
DEFAULT_WHEEL_RADIUS = 0.04
DEFAULT_LEFT_WHEEL_MOTOR_NAME = "wheel_left"
DEFAULT_RIGHT_WHEEL_MOTOR_NAME = "wheel_right"
DEFAULT_LEFT_WHEEL_ENCODER_NAME = "wheel_left_encoder"
DEFAULT_RIGHT_WHEEL_ENCODER_NAME = "wheel_right_encoder"

DEFAULT_ROBOT_BASE_NAME = "robot"

DEVICE_CONFIG = {
    'robot': {'publish_base_footprint': True},
    'distance_sens_right': {'always_publish': True},
    'distance_sens_left': {'always_publish': True},
    # 'lift_center': {'always_publish': False},
}

class WebotsDriver(WebotsDifferentialDriveNode):
    def __init__(self, args):
        super().__init__(
            "dipl",
            args,
            wheel_distance = DEFAULT_WHEEL_DISTANCE,
            wheel_radius = DEFAULT_WHEEL_RADIUS,
            left_joint=DEFAULT_LEFT_WHEEL_MOTOR_NAME,
            right_joint=DEFAULT_RIGHT_WHEEL_MOTOR_NAME,
            left_encoder=DEFAULT_LEFT_WHEEL_ENCODER_NAME,
            right_encoder=DEFAULT_RIGHT_WHEEL_ENCODER_NAME,
            command_topic='/cmd_vel',
            odometry_topic='/odom',
            odometry_frame='odom',
            robot_base_frame=DEFAULT_ROBOT_BASE_NAME
        )


        # self.get_logger().info('Differential driver init done')

        self.start_device_manager(DEVICE_CONFIG)
    #     self.counter = 0
        
    #     self.custom_publisher = self.create_publisher(String, 'custom_time', 1)

    #     self.create_timer(self.timestep / 1, self.__publish_time)
    # def __publish_distance_sensor_data(self):
    # def __publish_time(self):
    #     self.counter += 1
    #     self.custom_publisher.publish(String(data=f"Tick count {self.counter}"))


class LiftDriver(WebotsNode):
    def __init__(self, args, lift_name='lift_center', command_topic='/lift'):
        super().__init__("dipl", args)


        lift = self.declare_parameter('lift', lift_name)
        command_topic_param = self.declare_parameter('command_topic', command_topic)
        self.get_logger().info(f'Initializing lift drive node with name = {lift_name}')

        self.lift = self.robot.getMotor(lift.value)
        # self.lift.setPosition(0)
        # self.lift.setVelocity(0)
        self.create_subscription(Point, command_topic_param.value, self._cmd_lift_control, 1)

    def _cmd_lift_control(self, position):
        self.get_logger().info('Lift message received')
        self.get_logger().info(f'pos x = {position.x}')
        self.get_logger().info(f'pos y = {position.y}')
        self.get_logger().info(f'pos z = {position.z}')
        self.lift.setPosition(position.z)
        # right_velocity = twist.linear.x + self._wheel_distance * twist.angular.z / 2
        # left_velocity = twist.linear.x - self._wheel_distance * twist.angular.z / 2
        # left_omega = left_velocity / (self._wheel_radius)
        # right_omega = right_velocity / (self._wheel_radius)
        # self.left_motor.setVelocity(left_omega)
        # self.right_motor.setVelocity(right_omega)

def main(args=None):
    rclpy.init(args=args)

    webots_driver = WebotsDriver(args=args)
    lift_driver = LiftDriver(args=args)
    
    rclpy.spin(webots_driver, lift_driver)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
