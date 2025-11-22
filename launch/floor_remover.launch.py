from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('D435i_floor_remover')

    # Default config file path
    default_config = os.path.join(pkg_dir, 'config', 'floor_remover_params.yaml')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to config file for floor remover node'
    )

    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Use IMU for floor normal estimation'
    )

    input_cloud_topic_arg = DeclareLaunchArgument(
        'input_cloud_topic',
        default_value='/camera/depth/color/points',
        description='Input point cloud topic'
    )

    input_imu_topic_arg = DeclareLaunchArgument(
        'input_imu_topic',
        default_value='/camera/imu',
        description='Input IMU topic'
    )

    output_cloud_topic_arg = DeclareLaunchArgument(
        'output_cloud_topic',
        default_value='/floor_removed_cloud',
        description='Output point cloud topic'
    )

    # Floor remover node
    floor_remover_node = Node(
        package='D435i_floor_remover',
        executable='floor_remover_node',
        name='floor_remover_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_imu_for_normal': LaunchConfiguration('use_imu'),
                'input_cloud_topic': LaunchConfiguration('input_cloud_topic'),
                'input_imu_topic': LaunchConfiguration('input_imu_topic'),
                'output_cloud_topic': LaunchConfiguration('output_cloud_topic'),
            }
        ],
        remappings=[
            ('~/input', LaunchConfiguration('input_cloud_topic')),
            ('~/output', LaunchConfiguration('output_cloud_topic')),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_imu_arg,
        input_cloud_topic_arg,
        input_imu_topic_arg,
        output_cloud_topic_arg,
        floor_remover_node
    ])
