from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    map_pcd = DeclareLaunchArgument(
        "map_pcd",
        default_value="/home/huang/xxx/output/go2_built_map.pcd",
        description="Saved PCD map shown in RViz.",
    )
    map_topic = DeclareLaunchArgument(
        "map_topic",
        default_value="/go2_saved_map",
        description="PointCloud2 topic used to display the saved map.",
    )
    map_frame = DeclareLaunchArgument(
        "map_frame",
        default_value="odom",
        description="Frame used by the saved point cloud map and goal clicks.",
    )
    goal_topic = DeclareLaunchArgument(
        "goal_topic",
        default_value="/goal_pose",
        description="RViz 2D Goal Pose topic.",
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
    sport_request_topic = DeclareLaunchArgument(
        "sport_request_topic",
        default_value="/api/sport/request",
        description="Unitree sport request topic.",
    )
    rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Whether to launch RViz.",
    )

    map_publisher = Node(
        package="go2_navigation",
        executable="pcd_map_publisher",
        name="go2_saved_map_publisher",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "pcd_path": LaunchConfiguration("map_pcd"),
                "cloud_topic": LaunchConfiguration("map_topic"),
                "frame_id": LaunchConfiguration("map_frame"),
                "publish_period": 1.0,
            }
        ],
    )

    goal_controller = Node(
        package="go2_navigation",
        executable="simple_goal_controller",
        name="go2_simple_goal_controller",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "goal_topic": LaunchConfiguration("goal_topic"),
                "cmd_vel_topic": LaunchConfiguration("nav_cmd_topic"),
                "odom_topic": "/utlidar/robot_odom",
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
                "odom_topic": "/utlidar/robot_odom",
                "range_info_topic": "/utlidar/range_info",
                "cloud_topic": "/utlidar/cloud_deskewed",
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_click_nav",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("go2_navigation"), "rviz_cfg", "click_goal_nav.rviz"]
            ),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            params_file,
            map_pcd,
            map_topic,
            map_frame,
            goal_topic,
            nav_cmd_topic,
            cmd_vel_topic,
            sport_request_topic,
            rviz,
            map_publisher,
            goal_controller,
            obstacle_filter,
            sport_bridge,
            rviz_node,
        ]
    )
