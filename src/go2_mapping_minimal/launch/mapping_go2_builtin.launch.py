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

    named_pose_save_path_arg = DeclareLaunchArgument(
        'named_pose_save_path', default_value='/home/huang/xxx/output/go2_named_poses.json',
        description='JSON output file path for named poses recorded during mapping.')
    named_pose_gui_arg = DeclareLaunchArgument(
        'named_pose_gui', default_value='true',
        description='Whether to open the named-pose GUI panel during mapping.')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('go2_navigation'), 'config', 'go2_navigation.yaml']
        ),
        description='Shared YAML parameter file for navigation nodes.')
    goal_topic_arg = DeclareLaunchArgument(
        'goal_topic', default_value='/goal_pose',
        description='PoseStamped goal topic used by mapping-time navigation.')
    nav_cmd_topic_arg = DeclareLaunchArgument(
        'nav_cmd_topic', default_value='/cmd_vel_nav',
        description='Raw navigation velocity before obstacle filtering.')
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/cmd_vel',
        description='Velocity topic shared between navigation and the Unitree bridge.')
    height_map_topic_arg = DeclareLaunchArgument(
        'height_map_topic', default_value='/utlidar/height_map_array',
        description='Height-map topic used for terrain filtering during mapping.')
    sport_request_topic_arg = DeclareLaunchArgument(
        'sport_request_topic', default_value='/api/sport/request',
        description='Unitree sport request topic.')
    named_goal_json_arg = DeclareLaunchArgument(
        'named_goal_json', default_value='/home/huang/xxx/output/go2_named_poses.json',
        description='JSON file containing named navigation poses.')
    named_goal_gui_arg = DeclareLaunchArgument(
        'named_goal_gui', default_value='true',
        description='Whether to open the named-goal GUI during mapping.')

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

    named_pose_gui_node = Node(
        package='go2_navigation',
        executable='named_pose_gui',
        name='go2_named_pose_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('named_pose_gui')),
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
            'save_path': LaunchConfiguration('named_pose_save_path'),
            'window_title': 'Go2 Mapping Named Poses',
        }]
    )

    goal_controller_node = Node(
        package='go2_navigation',
        executable='simple_goal_controller',
        name='go2_simple_goal_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'odom_topic': LaunchConfiguration('odom_topic'),
                'goal_topic': LaunchConfiguration('goal_topic'),
                'cmd_vel_topic': LaunchConfiguration('nav_cmd_topic'),
            }
        ]
    )

    obstacle_filter_node = Node(
        package='go2_navigation',
        executable='dynamic_obstacle_filter',
        name='go2_dynamic_obstacle_filter',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'input_cmd_topic': LaunchConfiguration('nav_cmd_topic'),
                'output_cmd_topic': LaunchConfiguration('cmd_vel_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'range_info_topic': '/utlidar/range_info',
                'cloud_topic': LaunchConfiguration('cloud_topic'),
                'height_map_topic': LaunchConfiguration('height_map_topic'),
            }
        ]
    )

    sport_bridge_node = Node(
        package='go2_navigation',
        executable='unitree_sport_bridge',
        name='unitree_sport_bridge',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'sport_request_topic': LaunchConfiguration('sport_request_topic'),
            }
        ]
    )

    named_goal_gui_node = Node(
        package='go2_navigation',
        executable='named_goal_gui',
        name='go2_named_goal_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('named_goal_gui')),
        arguments=[
            '--json',
            LaunchConfiguration('named_goal_json'),
            '--topic',
            LaunchConfiguration('goal_topic'),
        ],
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
        named_pose_save_path_arg,
        named_pose_gui_arg,
        params_file_arg,
        goal_topic_arg,
        nav_cmd_topic_arg,
        cmd_vel_topic_arg,
        height_map_topic_arg,
        sport_request_topic_arg,
        named_goal_json_arg,
        named_goal_gui_arg,
        map_builder_node,
        named_pose_gui_node,
        goal_controller_node,
        obstacle_filter_node,
        sport_bridge_node,
        named_goal_gui_node,
        rviz_node,
    ])
