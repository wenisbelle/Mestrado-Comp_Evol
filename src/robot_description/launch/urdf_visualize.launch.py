import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import (Node, SetParameter)
import launch_ros.descriptions

import xacro

# this is the function launch  system will look for
def generate_launch_description():

    # Get Package Description and Directory #
    package_description = "robot_description"
    package_directory = get_package_share_directory(package_description)

    # Load URDF File #
    urdf_file = 'robot.xacro'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)
    print("URDF Loaded !")

    # Robot State Publisher (RSP) #
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output="both",
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 
                     'robot_description': launch_ros.descriptions.ParameterValue(Command(['xacro ', robot_desc_path]), value_type=str)}]
    )


    # create and return launch description object
    return LaunchDescription(
        [   
            SetParameter(name="use_sim_time", value=True),         
            robot_state_publisher_node,
        ]
    )