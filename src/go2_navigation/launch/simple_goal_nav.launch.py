from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("go2_navigation"), "config", "go2_navigation.yaml"]
        ),
        description="Shared YAML parameter file for navigation nodes.",
    )
    odom_topic = DeclareLaunchArgument(
        "odom_topic",
        default_value="/utlidar/robot_odom",
        description="Odometry topic used by the simple goal controller.",
    )
    goal_topic = DeclareLaunchArgument(
        "goal_topic",
        default_value="/goal_pose",
        description="PoseStamped goal topic.",
    )
    nav_cmd_topic = DeclareLaunchArgument(
        "nav_cmd_topic",
        default_value="/cmd_vel_nav",
        description="Raw navigation velocity before obstacle filtering.",
    )
    cmd_vel_topic = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="Velocity topic shared between the goal controller and the Unitree bridge.",
    )
    height_map_topic = DeclareLaunchArgument(
        "height_map_topic",
        default_value="/utlidar/height_map_array",
        description="Height-map topic used for terrain filtering.",
    )
    sport_request_topic = DeclareLaunchArgument(
        "sport_request_topic",
        default_value="/api/sport/request",
        description="Unitree sport request topic.",
    )

    goal_controller = Node(
        package="go2_navigation",
        executable="simple_goal_controller",
        name="go2_simple_goal_controller",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "odom_topic": LaunchConfiguration("odom_topic"),
                "goal_topic": LaunchConfiguration("goal_topic"),
                "cmd_vel_topic": LaunchConfiguration("nav_cmd_topic"),
            }
        ],
    )

    obstacle_filter = Node(
        package="go2_navigation",
        executable="dynamic_obstacle_filter",
        name="go2_dynamic_obstacle_filter",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "input_cmd_topic": LaunchConfiguration("nav_cmd_topic"),
                "output_cmd_topic": LaunchConfiguration("cmd_vel_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "range_info_topic": "/utlidar/range_info",
                "cloud_topic": "/utlidar/cloud_deskewed",
                "height_map_topic": LaunchConfiguration("height_map_topic"),
            }
        ],
    )

    sport_bridge = Node(
        package="go2_navigation",
        executable="unitree_sport_bridge",
        name="unitree_sport_bridge",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "sport_request_topic": LaunchConfiguration("sport_request_topic"),
            }
        ],
    )

    return LaunchDescription(
        [
            params_file,
            odom_topic,
            goal_topic,
            nav_cmd_topic,
            cmd_vel_topic,
            height_map_topic,
            sport_request_topic,
            goal_controller,
            obstacle_filter,
            sport_bridge,
        ]
    )
