from setuptools import find_packages, setup

package_name = "dexterous_hand_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/leap_hand.launch.py"]),
        ("share/" + package_name + "/config", ["config/ros2_params.yaml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    entry_points={"console_scripts": ["leap_hand_node = dexterous_hand_ros.node:main"]},
)
