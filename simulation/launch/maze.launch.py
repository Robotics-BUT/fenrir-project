"""Launch the Fenrir sim in the maze world (Phase 4 / Labs 10–13).

Brings up:
  * gz_sim with maze.sdf,
  * robot_state_publisher fed by xacro-rendered fenrir.urdf.xacro,
  * a spawn of the robot at the SW start cell (-1.4, -1.4) facing east,
  * ros_gz_bridge with the static topic map (config/ros_gz_bridge.yaml),
  * motor_bridge + line_sensor_bridge + lidar_bridge,
  * optional RViz2 (launch arg `rviz:=true`).

Usage:
    ros2 launch fenrir_sim maze.launch.py
    ros2 launch fenrir_sim maze.launch.py headless:=true rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share   = get_package_share_directory("fenrir_sim")
    urdf_xacro  = os.path.join(pkg_share, "description", "fenrir.urdf.xacro")
    bridge_yaml = os.path.join(pkg_share, "config",      "ros_gz_bridge.yaml")
    world_path  = os.path.join(pkg_share, "worlds",      "maze.sdf")

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="false",
        description="Open RViz2 alongside Gazebo.")
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Run gz_sim with -s (no GUI).")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use Gazebo simulation clock.")

    def _gz_sim(context):
        headless = LaunchConfiguration("headless").perform(context)
        flags = "-s -r" if headless.lower() == "true" else "-r"
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"),
                    "launch", "gz_sim.launch.py",
                ])
            ),
            launch_arguments={"gz_args": f"{flags} {world_path}"}.items(),
        )]
    gz_sim = OpaqueFunction(function=_gz_sim)

    robot_description = ParameterValue(
        Command(["xacro ", urdf_xacro]),
        value_type=str,
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    # Spawn in the SW start cell (cell (0,0) center) facing east.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "fenrir",
            "-topic", "robot_description",
            "-x", "-1.4", "-y", "-1.4", "-z", "0.05",
            "-Y", "0.0",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_yaml}"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    motor_bridge = Node(
        package="fenrir_sim",
        executable="motor_bridge",
        output="screen",
        parameters=[{
            "wheel_separation": 0.12,
            "watchdog_seconds": 1.0,
            "publish_period":   0.05,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    line_sensor_bridge = Node(
        package="fenrir_sim",
        executable="line_sensor_bridge",
        output="screen",
        parameters=[{
            "left_col_frac":  0.25,
            "right_col_frac": 0.75,
            "row_frac":       0.5,
            "sample_radius":  2,
            "max_reading":    1023,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    lidar_bridge = Node(
        package="fenrir_sim",
        executable="lidar_bridge",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    encoder_bridge = Node(
        package="fenrir_sim",
        executable="encoder_bridge",
        output="screen",
        parameters=[{
            "pulses_per_rev": 576,
            "publish_period": 0.01,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    gz_resource = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(pkg_share, "worlds")
        + ":" + os.path.join(pkg_share, "description"),
    )

    return LaunchDescription([
        rviz_arg,
        headless_arg,
        use_sim_time_arg,
        gz_resource,
        gz_sim,
        rsp,
        spawn,
        bridge,
        motor_bridge,
        line_sensor_bridge,
        lidar_bridge,
        encoder_bridge,
        rviz,
    ])
