#!/usr/bin/env python3
"""
Bring up Jim in Gazebo Harmonic with ros2_control.

    ros2 launch jim_bringup sim.launch.py

Startup ordering matters here and it is the single most common source of
"my controller won't load" errors. The sequence has to be:

    1. Gazebo starts
    2. Robot spawns into it, which starts the controller_manager INSIDE
       the simulator (that is what the gz_ros2_control plugin does)
    3. Only then can controllers be spawned

Spawning a controller before step 2 finishes fails, because there is no
controller_manager listening yet. The event handlers below enforce that
ordering instead of relying on sleeps.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.substitutions import EnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    desc_pkg = FindPackageShare("jim_description")
    bringup_pkg = FindPackageShare("jim_bringup")

    xacro_file = PathJoinSubstitution([desc_pkg, "urdf", "jim.urdf.xacro"])
    params_file = PathJoinSubstitution([bringup_pkg, "config", "controllers.yaml"])
    world_file = PathJoinSubstitution([bringup_pkg, "worlds", "jim_world.sdf"])
    rviz_file = PathJoinSubstitution([desc_pkg, "rviz", "jim.rviz"])

    # ParameterValue(..., value_type=str) is mandatory. Without it the
    # parameter layer tries to YAML-parse the URDF and dies on the XML.
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ", xacro_file,
                " use_sim:=true",
                " params_file:=", params_file,
            ]),
            value_type=str,
        )
    }

    gz_resources = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            PathJoinSubstitution([desc_pkg, ".."]), ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    # Gazebo publishes its clock on gz-transport, not on a ROS topic.
    # Without this bridge, every node with use_sim_time:=True waits
    # forever for a /clock that never arrives.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # ------------------------------------------------------------------ #
    # Gazebo
    # ------------------------------------------------------------------ #
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={"gz_args": [world_file, " -r -v1"]}.items(),
    )

    # ------------------------------------------------------------------ #
    # Robot state publisher - publishes TF from /joint_states
    # use_sim_time is essential. Without it RSP timestamps TF with wall
    # clock while Gazebo publishes joint states on sim clock, and RViz
    # shows nothing but extrapolation errors.
    # ------------------------------------------------------------------ #
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # ------------------------------------------------------------------ #
    # Spawn the robot from the description already on the topic
    # ------------------------------------------------------------------ #
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "jim"],
    )

    # ------------------------------------------------------------------ #
    # Controllers, chained so each waits for the previous
    # ------------------------------------------------------------------ #
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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),

        gz_resources,
        gz_sim,
        clock_bridge,
        rsp,
        spawn,

        RegisterEventHandler(
            OnProcessExit(target_action=spawn, on_exit=[jsb])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=jsb, on_exit=[gantry, rviz])
        ),
    ])
