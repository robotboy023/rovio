import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    package_share = FindPackageShare('rovio').find('rovio')
    config_file = os.path.join(package_share, 'cfg', 'rovio.info')
    filter_config_arg = DeclareLaunchArgument('filter_config',
                                              default_value=config_file)
    cam0_config = os.path.join(package_share,'cfg', 'euroc_cam0.yaml')
    cam0_config_arg = DeclareLaunchArgument('cam0_config',default_value=cam0_config)
    cam1_config = os.path.join(package_share,'cfg', 'euroc_cam1.yaml')
    cam1_config_arg = DeclareLaunchArgument('cam1_config',default_value=cam1_config)
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value="/imu0")
    cam0_topic_arg = DeclareLaunchArgument('cam0_topic', default_value="/cam0/image_raw")
    cam1_topic_arg = DeclareLaunchArgument('cam1_topic', default_value="/cam1/image_raw")
    resize_image_arg = DeclareLaunchArgument('resize_image', default_value=False)
    resize_image_width_arg = DeclareLaunchArgument('resize_image_width', default_value=320)
    resize_image_height_arg = DeclareLaunchArgument('resize_image_height', default_value=240)

    print("cam1 config: ", cam1_config)
    print("cam0 config: ", cam0_config)
    print("filter config: ", config_file)
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
                'resize_image' : LaunchConfiguration('resize_image'),
                'resize_image_width': LaunchConfiguration('resize_image_width'),
                'resize_image_height': LaunchConfiguration('resize_image_height'),
                'use_sim_time' : True
            }
        ]
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
        rovio_node
    ])
