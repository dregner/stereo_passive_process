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

        DeclareLaunchArgument(
            'slam',
            default_value='false',
            description='Enable SLAM'),
        
        DeclareLaunchArgument(
            'disparity',
            default_value='false',
            description='Enable Disparity'),

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
                    'disparity_enabled': LaunchConfiguration('disparity')
                }
            ],
        ),
    ])
