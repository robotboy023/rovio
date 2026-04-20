import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_share = FindPackageShare('rovio').find('rovio')
    config_file = os.path.join(package_share, 'cfg', 'rovio.info')
    filter_config_arg = DeclareLaunchArgument('filter_config', default_value=config_file)
    cam0_config = os.path.join(package_share, 'cfg', 'euroc_cam0.yaml')
    cam0_config_arg = DeclareLaunchArgument('cam0_config', default_value=cam0_config)
    cam1_config = os.path.join(package_share, 'cfg', 'euroc_cam1.yaml')
    cam1_config_arg = DeclareLaunchArgument('cam1_config', default_value=cam1_config)
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value="/imu0")
    cam0_topic_arg = DeclareLaunchArgument('cam0_topic', default_value="/cam0/image_raw")
    cam1_topic_arg = DeclareLaunchArgument('cam1_topic', default_value="/cam1/image_raw")
    resize_image_arg = DeclareLaunchArgument('resize_image', default_value="false")
    resize_image_width_arg = DeclareLaunchArgument('resize_image_width', default_value="320")
    resize_image_height_arg = DeclareLaunchArgument('resize_image_height', default_value="240")
    vis_fps_arg = DeclareLaunchArgument('vis_fps', default_value="5")
    velocity_threshold_arg = DeclareLaunchArgument('health_tracker_velocity_threshold',
                                                   default_value="1.0")
    accel_threshold_arg = DeclareLaunchArgument('health_tracker_accel_threshold',
                                                default_value="0.5")
    pixel_cov_threshold_arg = DeclareLaunchArgument('health_tracker_pixel_cov_threshold',
                                                    default_value="10.0")
    map_frame_arg = DeclareLaunchArgument('map_frame', default_value="/map")
    world_frame_arg = DeclareLaunchArgument('world_frame', default_value="/world")
    camera_frame_arg = DeclareLaunchArgument('camera_frame', default_value="/camera")
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value="/imu")

    rovio_node = Node(
        package='rovio',
        executable='rovio_node',
        name='rovio',
        output='screen',
        parameters=[
            {
                'filter_config': LaunchConfiguration('filter_config'),
                'camera0_config': LaunchConfiguration('cam0_config'),
                'camera1_config': LaunchConfiguration('cam1_config'),
                'imu_topic' : LaunchConfiguration('imu_topic'),
                'cam0_topic' : LaunchConfiguration('cam0_topic'),
                'cam1_topic' : LaunchConfiguration('cam1_topic'),
                'use_sim_time' : True,
                'resize_image' : LaunchConfiguration('resize_image'),
                'resize_image_width': LaunchConfiguration('resize_image_width'),
                'resize_image_height': LaunchConfiguration('resize_image_height'),
                'vis_fps': LaunchConfiguration('vis_fps'),
                'health_tracker_velocity_threshold':
                    LaunchConfiguration('health_tracker_velocity_threshold'),
                'health_tracker_accel_threshold':
                    LaunchConfiguration('health_tracker_accel_threshold'),
                'health_tracker_pixel_cov_threshold':
                    LaunchConfiguration('health_tracker_pixel_cov_threshold'),
                'map_frame': LaunchConfiguration('map_frame'),
                'world_frame': LaunchConfiguration('world_frame'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'imu_frame': LaunchConfiguration('imu_frame')
            }
        ]
    )

    rovio_image_view_node = Node(
        package='image_view',
        executable='image_view',
        name='rovio_image_view',
        remappings=[('image', '/rovio/imgVis0')]
    )

    return LaunchDescription([
        filter_config_arg,
        cam0_config_arg,
        cam1_config_arg,
        imu_topic_arg,
        cam0_topic_arg,
        cam1_topic_arg,
        resize_image_arg,
        resize_image_width_arg,
        resize_image_height_arg,
        vis_fps_arg,
        velocity_threshold_arg,
        accel_threshold_arg,
        pixel_cov_threshold_arg,
        map_frame_arg,
        world_frame_arg,
        camera_frame_arg,
        imu_frame_arg,
        rovio_node,
        rovio_image_view_node
    ])
