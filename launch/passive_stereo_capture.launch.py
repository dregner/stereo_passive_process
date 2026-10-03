from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg = FindPackageShare('passive_stereo_capture')

    default_params = PathJoinSubstitution([pkg, 'config', 'bfs_04S4C.yaml'])
    default_calib  = PathJoinSubstitution(
        [pkg, 'config', 'stereo_calibration_bfs.yaml'])
    default_params = '/home/jetson/ros2_ws/src/stereo_passive_process/config/bfs_04S4C.yaml'
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the passive_stereo YAML parameter file'),

        DeclareLaunchArgument(
            'calibration_file',
            default_value=default_calib,
            description='Path to the OpenCV stereo calibration YAML'),

        DeclareLaunchArgument(
            'slam',        default_value='true',
            description='Enable SLAM (ORB-SLAM3)'),

        DeclareLaunchArgument(
            'disparity',   default_value='true',
            description='Enable Retinify GPU disparity'),

        DeclareLaunchArgument(
            'preview',     default_value='true',
            description='Enable JPEG preview streaming'),

        DeclareLaunchArgument(
            'trigger',     default_value='true',
            description='Use hardware GPIO trigger (true) or continuous (false)'),

        DeclareLaunchArgument(
            'pangolin',    default_value='false',
            description='Open Pangolin viewer in SLAM (requires X11)'),

        Node(
            package='passive_stereo_capture',
            executable='passive_stereo_node',
            name='passive_stereo_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'calibration_file':  LaunchConfiguration('calibration_file'),
                    'slam_enabled':      LaunchConfiguration('slam'),
                    'disparity_enabled': LaunchConfiguration('disparity'),
                    'preview_enabled':   LaunchConfiguration('preview'),
                    'trigger_mode':      LaunchConfiguration('trigger'),
                    'slam_use_pangolin': LaunchConfiguration('pangolin'),
                }
            ],
        ),
    ])
