from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("go2_camera_bridge"), "config", "go2_camera_bridge.yaml"]
        ),
        description="YAML parameter file for the Go2 camera bridge node.",
    )
    network_interface = DeclareLaunchArgument(
        "network_interface",
        default_value="",
        description="Ethernet interface connected to the Go2 robot. Empty enables auto-detection.",
    )
    fps = DeclareLaunchArgument(
        "fps",
        default_value="15.0",
        description="Polling rate used for the Unitree SDK image requests.",
    )
    publish_raw = DeclareLaunchArgument(
        "publish_raw",
        default_value="true",
        description="Whether to publish sensor_msgs/Image.",
    )
    publish_compressed = DeclareLaunchArgument(
        "publish_compressed",
        default_value="true",
        description="Whether to publish sensor_msgs/CompressedImage.",
    )

    camera_bridge = Node(
        package="go2_camera_bridge",
        executable="go2_front_camera_bridge",
        name="go2_front_camera_bridge",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "network_interface": LaunchConfiguration("network_interface"),
                "fps": LaunchConfiguration("fps"),
                "publish_raw": LaunchConfiguration("publish_raw"),
                "publish_compressed": LaunchConfiguration("publish_compressed"),
            },
        ],
    )

    return LaunchDescription(
        [
            params_file,
            network_interface,
            fps,
            publish_raw,
            publish_compressed,
            camera_bridge,
        ]
    )
