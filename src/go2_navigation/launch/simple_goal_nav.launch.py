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
        description="Odometry topic used by the EGO local planner.",
    )
    path_topic = DeclareLaunchArgument(
        "path_topic",
        default_value="/pct_path",
        description="nav_msgs/Path topic used as the reference path for EGO planning.",
    )
    cloud_topic = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/utlidar/cloud_deskewed",
        description="PointCloud2 topic used for local occupancy updates.",
    )
    cmd_vel_topic = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="Velocity topic shared between the trajectory tracker and the Unitree bridge.",
    )
    sport_request_topic = DeclareLaunchArgument(
        "sport_request_topic",
        default_value="/api/sport/request",
        description="Unitree sport request topic.",
    )

    planner_node = Node(
        package="ego_planner",
        executable="ego_planner_node",
        name="ego_planner_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
        ],
        remappings=[
            ("odom_world", LaunchConfiguration("odom_topic")),
            ("grid_map/odom", LaunchConfiguration("odom_topic")),
            ("grid_map/cloud", LaunchConfiguration("cloud_topic")),
            ("reference_path", LaunchConfiguration("path_topic")),
        ],
    )

    traj_server = Node(
        package="ego_planner",
        executable="go2_traj_server",
        name="go2_traj_server",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "odom_topic": LaunchConfiguration("odom_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            }
        ],
        remappings=[
            ("planning/bspline", "planning/bspline"),
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

    gait_init_node = Node(
        package="go2_navigation",
        executable="gait_init_node",
        name="gait_init_node",
        output="screen",
        parameters=[
            {
                "sport_request_topic": LaunchConfiguration("sport_request_topic"),
            }
        ],
    )

    return LaunchDescription(
        [
            params_file,
            odom_topic,
            path_topic,
            cloud_topic,
            cmd_vel_topic,
            sport_request_topic,
            gait_init_node,
            planner_node,
            traj_server,
            sport_bridge,
        ]
    )
