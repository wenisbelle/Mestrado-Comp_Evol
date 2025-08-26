#!/usr/bin/python3
# -*- coding: utf-8 -*-
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch import LaunchDescription


# this is the function launch  system will look for


def generate_launch_description():


    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name=[robot_name, '_joint_state_broadcaster'],
        namespace=robot_name,
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_wheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        name='robot01_base_controller',
        namespace=robot_name,
        arguments=["base_controller"],
        output="screen",
    )
    


    # create and return launch description object
    return LaunchDescription(
        [
            spawn_broadcaster,
            spawn_wheel_controller
        ]
    )