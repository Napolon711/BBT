from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('ros2_libxr'), 'config', 'bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='STM32 USB CDC 串口设备'),
        DeclareLaunchArgument('own_team',
                              description='开局人工指定己方 red 或 blue；不核对裁判 ID'),
        DeclareLaunchArgument('config', default_value=default_config),
        Node(
            package='ros2_libxr',
            executable='ros2_libxr_node',
            name='ros2_libxr_bridge',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {
                    'port': LaunchConfiguration('port'),
                    'own_team': LaunchConfiguration('own_team'),
                },
            ],
        ),
    ])
