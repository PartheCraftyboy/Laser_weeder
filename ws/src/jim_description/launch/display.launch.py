#!/usr/bin/env python3
"""
Load Jim's description and show it in RViz with joint sliders.

    ros2 launch jim_description display.launch.py

This is the "does my model make sense" launch file. No Gazebo, no
controllers, no physics - just the URDF, TF, and a slider panel so you
can drag the joints and watch the carriages move.

If the model looks right here, the description is correct and every
problem after this point is a controller or simulation problem, not a
URDF problem. That separation saves a lot of debugging time.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare("jim_description")

    xacro_file = PathJoinSubstitution([pkg, "urdf", "jim.urdf.xacro"])
    rviz_file = PathJoinSubstitution([pkg, "rviz", "jim.rviz"])

    # xacro runs at launch time and expands to plain URDF. The space after
    # "xacro" matters - Command joins these without adding one.
    robot_description = {
        "robot_description": Command(["xacro ", xacro_file])
    }

    return LaunchDescription([

        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Show the joint slider panel",
        ),

        # Publishes TF for every link, using joint values from /joint_states.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),

        # Fakes /joint_states from slider positions. In simulation this is
        # replaced by gz_ros2_control; on hardware, by the real driver.
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_file],
        ),
    ])
