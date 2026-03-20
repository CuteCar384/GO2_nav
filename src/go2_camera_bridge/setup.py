from setuptools import find_packages, setup


package_name = "go2_camera_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/front_camera_bridge.launch.py"]),
        ("share/" + package_name + "/config", ["config/go2_camera_bridge.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="huang",
    maintainer_email="huang@example.com",
    description="ROS 2 bridge package for Unitree Go2 front camera streams.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "go2_front_camera_bridge = go2_camera_bridge.front_camera_bridge:main",
        ],
    },
)
