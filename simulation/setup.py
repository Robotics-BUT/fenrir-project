"""Setup script for the fenrir_sim package (Phase 4)."""

from glob import glob
import os

from setuptools import setup

package_name = "fenrir_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "worlds"),
         glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "description"),
         glob("description/*.urdf.xacro")),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Adam Ligocki",
    maintainer_email="adam.ligocki@vut.cz",
    description="Fenrir Gazebo Harmonic simulation: URDF + worlds + bridge.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motor_bridge       = fenrir_sim.motor_bridge:main",
            "line_sensor_bridge = fenrir_sim.line_sensor_bridge:main",
            "lidar_bridge       = fenrir_sim.lidar_bridge:main",
        ],
    },
)
