from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def make_container(context):
    namespace = LaunchConfiguration('stereo_namespace').perform(context).strip('/')
    stereo_namespace = f'/{namespace}' if namespace else '/'
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    approximate_sync = ParameterValue(
        LaunchConfiguration('approximate_sync'), value_type=bool
    )
    ipc = [{'use_intra_process_comms': True}]
    components = []

    for side in ('left', 'right'):
        camera_namespace = f"{stereo_namespace.rstrip('/')}/{side}"
        camera_info = LaunchConfiguration(f'{side}_camera_info')
        debayer_parameters = {'use_sim_time': use_sim_time}
        # RectifyNode uses the image publisher's QoS for its CameraInfo subscription.
        # Best effort lets it receive calibration from either simulator QoS profile.
        for image_topic in ('image_mono', 'image_color'):
            reliability = f'qos_overrides.{camera_namespace}/{image_topic}.publisher.reliability'
            debayer_parameters[reliability] = 'best_effort'
        components.append(ComposableNode(
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name='debayer',
            namespace=camera_namespace,
            parameters=[debayer_parameters],
            remappings=[('image_raw', LaunchConfiguration(f'{side}_image_raw'))],
            extra_arguments=ipc,
        ))

        # Disparity uses both mono images. The cloud also uses left rectified color.
        rectifiers = [('mono', 'image_mono', 'image_rect')]
        if side == 'left':
            rectifiers.append(('color', 'image_color', 'image_rect_color'))
        for name, image_in, image_out in rectifiers:
            components.append(ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name=f'rectify_{name}',
                namespace=camera_namespace,
                parameters=[{'use_sim_time': use_sim_time}],
                remappings=[
                    ('image', image_in),
                    ('camera_info', camera_info),
                    ('image_rect', image_out),
                ],
                extra_arguments=ipc,
            ))

    stereo_parameters = {
        'use_sim_time': use_sim_time,
        'approximate_sync': approximate_sync,
    }
    stereo_remappings = [
        ('left/camera_info', LaunchConfiguration('left_camera_info')),
        ('right/camera_info', LaunchConfiguration('right_camera_info')),
    ]
    components.extend([
        ComposableNode(
            package='stereo_image_proc',
            plugin='stereo_image_proc::DisparityNode',
            name='disparity',
            namespace=stereo_namespace,
            parameters=[stereo_parameters],
            remappings=stereo_remappings,
            extra_arguments=ipc,
        ),
        ComposableNode(
            package='stereo_image_proc',
            plugin='stereo_image_proc::PointCloudNode',
            name='point_cloud',
            namespace=stereo_namespace,
            parameters=[stereo_parameters, {
                'avoid_point_cloud_padding': True,
                'use_color': True,
            }],
            remappings=stereo_remappings,
            extra_arguments=ipc,
        ),
    ])

    return [ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_mt',
        name='stereo_image_proc_container',
        namespace=stereo_namespace,
        composable_node_descriptions=components,
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'stereo_namespace', default_value='/Passive/stereo',
            description='Namespace for rectified images, disparity and points2.',
        ),
        DeclareLaunchArgument(
            'left_image_raw', default_value='/Passive/left/image_raw',
            description='Absolute left raw image topic from the simulator.',
        ),
        DeclareLaunchArgument(
            'right_image_raw', default_value='/Passive/right/image_raw',
            description='Absolute right raw image topic from the simulator.',
        ),
        DeclareLaunchArgument(
            'left_camera_info', default_value='/Passive/left/camera_info',
            description='Absolute calibrated left CameraInfo topic.',
        ),
        DeclareLaunchArgument(
            'right_camera_info', default_value='/Passive/right/camera_info',
            description='Absolute calibrated right CameraInfo topic with the stereo baseline.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            choices=['true', 'false'], description='Use the simulation clock.',
        ),
        DeclareLaunchArgument(
            'approximate_sync', default_value='false', choices=['true', 'false'],
            description='Allow timestamp differences between left and right cameras.',
        ),
        OpaqueFunction(function=make_container),
    ])
