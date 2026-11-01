from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('ros2_libxr'), 'config', 'bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='STM32 USB CDC 串口设备'),
        DeclareLaunchArgument('robot_id', default_value='0',
                              description='哨兵机器人 ID，0 表示自动识别'),
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
                    'robot_id': ParameterValue(
                        LaunchConfiguration('robot_id'), value_type=int),
                },
            ],
        ),
    ])
