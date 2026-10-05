from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('passive_stereo_capture')

    default_params = PathJoinSubstitution([pkg, 'config', 'passive_stereo.yaml'])
    default_calib  = PathJoinSubstitution([pkg, 'config', 'stereo_calibration_bfs.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the passive_stereo YAML parameter file'),

        DeclareLaunchArgument(
            'calibration_file',
            default_value=default_calib,
            description='Path to the OpenCV stereo calibration YAML'),

        Node(
            package='passive_stereo_capture',
            executable='passive_stereo_node',
            name='passive_stereo_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'calibration_file': LaunchConfiguration('calibration_file'),
                }
            ],
        ),
    ])
