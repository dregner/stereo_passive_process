from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='/home/jetson/ros2_ws/src/stereo_passive_process/config/passive_stereo_lab.yaml',
            description='Path to the passive_stereo YAML parameter file'),

        DeclareLaunchArgument(
            'calibration_file',
            default_value= PathJoinSubstitution([FindPackageShare('passive_stereo_capture'),
                                                  'config', 'stereo_calibration_lab.yaml']),
            description='Path to the OpenCV stereo calibration YAML'),

        DeclareLaunchArgument('slam', default_value='false', description='Enable SLAM'),

        DeclareLaunchArgument('disparity', default_value='false', description='Enable Disparity'),

        DeclareLaunchArgument('mavlink_enabled', default_value='false',  description='Open the MAVLink bridge inside the capture process'),
        DeclareLaunchArgument('mavlink_bridge_url', default_value='udp://:14555@127.0.0.1:14550', description='libmavconn UDP, TCP or serial connection URL'),
        DeclareLaunchArgument('mavlink_message', default_value='estimate', choices=['estimate', 'delta'], description='Vision MAVLink message type'),

        Node(
            package='passive_stereo_capture',
            executable='passive_stereo_node',
            name='passive_stereo_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'calibration_file': LaunchConfiguration('calibration_file'),
                    'slam_enabled': LaunchConfiguration('slam'),
                    'disparity_enabled': LaunchConfiguration('disparity'),
                    'mavlink_enabled': LaunchConfiguration('mavlink_enabled'),
                    'mavlink_bridge_url': LaunchConfiguration('mavlink_bridge_url'),
                    'mavlink_message': LaunchConfiguration('mavlink_message')
                }
            ],
        ),
    ])
