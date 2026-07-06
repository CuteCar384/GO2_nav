from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic",
                default_value="/utlidar/height_map_array",
                description="Unitree HeightMap input topic.",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/utlidar/height_map_viz",
                description="PointCloud2 output topic for RViz2.",
            ),
            DeclareLaunchArgument(
                "sentinel_threshold",
                default_value="100000000.0",
                description="Cell values with abs(value) >= threshold are treated as invalid.",
            ),
            Node(
                package="go2_navigation",
                executable="height_map_viz",
                name="height_map_viz",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "sentinel_threshold": LaunchConfiguration("sentinel_threshold"),
                    }
                ],
            ),
        ]
    )
