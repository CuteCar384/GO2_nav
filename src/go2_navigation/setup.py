from setuptools import find_packages, setup


package_name = "go2_navigation"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/simple_goal_nav.launch.py",
                "launch/click_goal_nav.launch.py",
            ],
        ),
        ("share/" + package_name + "/config", ["config/go2_navigation.yaml"]),
        ("share/" + package_name + "/rviz_cfg", ["rviz_cfg/click_goal_nav.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="huang",
    maintainer_email="huang@example.com",
    description="Goal-point navigation helpers for Unitree Go2 based on ROS 2 topics and the Unitree Sport API.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dynamic_obstacle_filter = go2_navigation.dynamic_obstacle_filter:main",
            "simple_goal_controller = go2_navigation.simple_goal_controller:main",
            "unitree_sport_bridge = go2_navigation.unitree_sport_bridge:main",
            "publish_goal = go2_navigation.publish_goal:main",
            "pcd_map_publisher = go2_navigation.pcd_map_publisher:main",
        ],
    },
)
