import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution)
from launch_ros.actions import (Node, SetParameter)
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument


import xacro

# this is the function launch  system will look for
def generate_launch_description():

    declare_robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="robot",
        description="Name of the robot to launch"
    )
    robot_name = LaunchConfiguration("robot_name")

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
        namespace=robot_name,
        output="both",
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 
                     'robot_description': launch_ros.descriptions.ParameterValue(Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name]), value_type=str)}],
    )

    # Spawn the Robot #
    declare_spawn_x = DeclareLaunchArgument("x", default_value="-10.0",
                                            description="Model Spawn X Axis Value")
    declare_spawn_y = DeclareLaunchArgument("y", default_value="0.0",
                                            description="Model Spawn Y Axis Value")
    declare_spawn_z = DeclareLaunchArgument("z", default_value="0.5",
                                            description="Model Spawn Z Axis Value")
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="robot_spawn_entity",
        arguments=[
            "-name", robot_name,
            "-allow_renaming", "true",
            "-topic", [robot_name, '/robot_description'],
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )

    # ROS-Gazebo Bridge #
    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=[robot_name,"_ign_bridge"],
        arguments=[
            #[robot_name, "/clock"] + "@rosgraph_msgs/msg/Clock" + "@ignition.msgs.Clock",
            #"/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
            ['/model/', robot_name, '/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
            ['/', robot_name, '_lidar/laser/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
            ['/world/empty/model/', robot_name, '/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'],
                      
        ],
        remappings=[
            #(['/world/empty/model/', robot_name, '/joint_state'], 'joint_states'),
            (['/model/', robot_name, '/odometry'],[robot_name, '_true_odom']),
        ],
        output="screen",
    )

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
        name=[robot_name, '_base_controller'],
        namespace=robot_name,
        arguments=["base_controller"],
        output="screen",
    )


    # create and return launch description object
    return LaunchDescription(
        [   
            SetParameter(name="use_sim_time", value=True),
            declare_robot_name_arg,         
            robot_state_publisher_node,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_z,
            gz_spawn_entity,
            ign_bridge,
            spawn_broadcaster,
            spawn_wheel_controller
        ]
    )