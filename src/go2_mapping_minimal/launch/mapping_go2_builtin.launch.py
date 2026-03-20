from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to start RViz.')

    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic', default_value='/utlidar/cloud_deskewed',
        description='Deskewed cloud topic from the robot.')

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/utlidar/robot_odom',
        description='Built-in robot odometry topic.')

    save_path_arg = DeclareLaunchArgument(
        'save_path', default_value='/home/huang/xxx/output/go2_built_map.pcd',
        description='PCD output file path.')

    map_builder_node = Node(
        package='go2_mapping_minimal',
        executable='go2_map_builder',
        name='go2_map_builder',
        output='screen',
        parameters=[{
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'map_topic': '/go2_built_map',
            'path_topic': '/go2_robot_path',
            'voxel_leaf_size': 0.10,
            'publish_every_n_scans': 5,
            'downsample_every_n_scans': 10,
            'save_path': LaunchConfiguration('save_path'),
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('go2_mapping_minimal'),
            'rviz_cfg', 'go2_builtin.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )

    return LaunchDescription([
        rviz_arg,
        cloud_topic_arg,
        odom_topic_arg,
        save_path_arg,
        map_builder_node,
        rviz_node,
    ])
