import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('dipl')
    # world = LaunchConfiguration('world')
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
        ),
        launch_arguments=[
            ('package', 'dipl'),
            ('executable', 'driver'),
            ('world', 'install/dipl/share/dipl/worlds/world_v1.wbt'),
        ]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='epuck_world.wbt',
            description='Choose one of the world files from `/webots_ros2_epuck/world` directory'
        ),
        webots
    ])
    # return LaunchDescription([
    #     webots
    # ])