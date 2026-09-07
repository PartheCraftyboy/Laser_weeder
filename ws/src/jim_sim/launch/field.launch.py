#!/usr/bin/env python3
"""
Full phase-3 demo.

    ros2 launch jim_sim field.launch.py

Brings up the crop field in Gazebo, starts the controllers, runs the
oracle detector and the targeting node, and opens RViz with detection
and beam markers. The gantry then works through the weeds on its own.

Launch args:
    auto:=false      load everything but do not start cycling; drive it
                     manually with the ~/treat_next service
    rviz:=false      headless-ish, Gazebo GUI only
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# --------------------------------------------------------------------------
# work_surface: the soil plane under the gantry, origin at the centre of the
# reachable area. Every weed position is expressed here, which keeps the
# targeting maths free of CAD offsets.
#
# Derived from the URDF:
#   beam ground point (base_link) = (0.058675 + jx,  -0.126615 - jy,  -0.30)
#   centre of travel is jx = jy = 0.125
#   ->  (0.1837, -0.2516, -0.30)
# --------------------------------------------------------------------------
WORK_SURFACE_XYZ = ["0.1837", "-0.2516", "-0.30"]


def generate_launch_description():

    desc_pkg = FindPackageShare("jim_description")
    bringup_pkg = FindPackageShare("jim_bringup")
    sim_pkg = FindPackageShare("jim_sim")

    xacro_file = PathJoinSubstitution([desc_pkg, "urdf", "jim.urdf.xacro"])
    params_file = PathJoinSubstitution([bringup_pkg, "config", "controllers.yaml"])
    world_file = PathJoinSubstitution([sim_pkg, "worlds", "jim_field.sdf"])
    truth_file = PathJoinSubstitution([sim_pkg, "config", "ground_truth.yaml"])
    rviz_file = PathJoinSubstitution([sim_pkg, "config", "field.rviz"])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file,
                     " use_sim:=true",
                     " params_file:=", params_file]),
            value_type=str,
        )
    }

    # Gazebo cannot read the ament index, so package:// means nothing to it.
    gz_resources = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[PathJoinSubstitution([desc_pkg, ".."]), ":",
               EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={"gz_args": [world_file, " -r -v1"]}.items(),
    )

    # Gazebo publishes its clock on gz-transport, not DDS. Without this
    # bridge every use_sim_time node waits forever.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "jim"],
    )

    work_surface_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "--x", WORK_SURFACE_XYZ[0],
            "--y", WORK_SURFACE_XYZ[1],
            "--z", WORK_SURFACE_XYZ[2],
            "--frame-id", "base_link",
            "--child-frame-id", "work_surface",
        ],
    )

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    gantry = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gantry_controller"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    oracle = Node(
        package="jim_sim",
        executable="oracle_detector",
        name="oracle_detector",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "ground_truth_file": truth_file,
            "frame_id": "work_surface",
            "rate_hz": 2.0,
            "position_noise_std": 0.0015,
            "miss_rate": 0.0,
            "false_positive_rate": 0.0,
        }],
    )

    targeting = Node(
        package="jim_sim",
        executable="targeting_node",
        name="targeting_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "travel_x": 0.250,
            "travel_y": 0.250,
            "margin": 0.002,
            "max_velocity": 0.035,
            "min_confidence": 0.6,
            "auto_cycle": LaunchConfiguration("auto"),
            "dwell_s": 2.0,
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("auto", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),

        gz_resources,
        gz_sim,
        clock_bridge,
        rsp,
        work_surface_tf,
        spawn,

        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(
            OnProcessExit(target_action=jsb,
                          on_exit=[gantry, oracle, targeting, rviz])),
    ])
