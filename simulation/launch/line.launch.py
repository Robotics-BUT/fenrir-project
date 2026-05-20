"""Launch the Fenrir sim in the line-following world (Phase 4 vertical slice).

Brings up:
  * gz_sim with the line.sdf world,
  * robot_state_publisher fed by xacro-rendered fenrir.urdf.xacro,
  * a spawn of the robot into the gz world,
  * ros_gz_bridge with the static topic map (config/ros_gz_bridge.yaml),
  * the motor_bridge node that handles /bpc_prp_robot/set_motor_speeds,
  * optional RViz2 (launch arg `rviz:=true`).

Usage from inside the bpc-prp-sim:jazzy container:
    ros2 launch fenrir_sim line.launch.py
    ros2 launch fenrir_sim line.launch.py rviz:=true
    ros2 launch fenrir_sim line.launch.py headless:=true   # no gz-gui
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
    pkg_share = get_package_share_directory("fenrir_sim")
    world_path = os.path.join(pkg_share, "worlds", "line.sdf")
    urdf_xacro = os.path.join(pkg_share, "description", "fenrir.urdf.xacro")
    bridge_yaml = os.path.join(pkg_share, "config", "ros_gz_bridge.yaml")

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="false",
        description="Open RViz2 alongside Gazebo.")
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Run gz_sim with -s -r (no GUI).")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use Gazebo simulation clock.")

    # ---- gz_sim ------------------------------------------------------------
    # Compose gz_args at launch time: `-r` to start running immediately,
    # plus `-s` (server-only / no GUI) when `headless:=true`.
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

    # ---- robot_state_publisher --------------------------------------------
    # xacro renders the URDF at launch time so dimension tweaks don't need
    # a full rebuild. The ParameterValue(value_type=str) wrap is mandatory:
    # without it, launch tries to parse the rendered XML as YAML and dies.
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

    # ---- spawn the robot in the gz world ----------------------------------
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "fenrir",
            "-topic", "robot_description",
            "-x", "-1.5", "-y", "-1.0", "-z", "0.05",
            "-Y", "0.0",   # facing +x along the bottom straight
        ],
        output="screen",
    )

    # ---- ros_gz_bridge: native gz <-> ROS 2 topic types -------------------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p",
                   f"config_file:={bridge_yaml}"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    # ---- motor_bridge: /bpc_prp_robot/set_motor_speeds -> /cmd_vel --------
    motor_bridge = Node(
        package="fenrir_sim",
        executable="motor_bridge",
        output="screen",
        parameters=[{
            "wheel_separation": 0.20,
            "watchdog_seconds": 1.0,
            "publish_period":   0.05,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    # ---- line_sensor_bridge: floor_camera -> /bpc_prp_robot/line_sensors --
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

    # ---- optional RViz2 ---------------------------------------------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    # ---- env: tell gz where to find the world / model meshes --------------
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
        rviz,
    ])
