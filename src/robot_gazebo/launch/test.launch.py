import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
import launch_ros.descriptions
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # --- ARGUMENTS ---
    # Declare a new launch argument for the robot's name, with 'robot' as the default value.
    declare_robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="robot01",
        description="Name of the robot to launch"
    )

    # Declare arguments for the robot's spawn position.
    declare_spawn_x_arg = DeclareLaunchArgument("x", default_value="0.0", description="Model Spawn X Axis Value")
    declare_spawn_y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Model Spawn Y Axis Value")
    declare_spawn_z_arg = DeclareLaunchArgument("z", default_value="0.5", description="Model Spawn Z Axis Value")

    # --- CONFIGURATIONS ---
    # Get the value of the robot_name argument.
    robot_name = LaunchConfiguration("robot_name")
    x_val = LaunchConfiguration("x")
    y_val = LaunchConfiguration("y")
    z_val = LaunchConfiguration("z")

    # --- PATHS ---
    package_description = "robot_description"
    package_directory = get_package_share_directory(package_description)
    urdf_file = 'robot.xacro'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)

    # --- NODES ---
    # We'll group all robot-specific nodes under a namespace. This is crucial for multi-robot systems.
    robot_nodes = GroupAction(
        actions=[
            # Push the robot's namespace. All nodes within this GroupAction will be prefixed with /<robot_name>.
            PushRosNamespace(robot_name),

            # Robot State Publisher (RSP)
            # The name and namespace are now tied to the robot_name.
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output="both",
                emulate_tty=True,
                parameters=[{'use_sim_time': True,
                             'robot_description': launch_ros.descriptions.ParameterValue(
                                 Command(['xacro ', robot_desc_path]), value_type=str)
                            }]
            ),

            # Spawn the Robot in Gazebo
            # The '-name' argument now uses the robot_name LaunchConfiguration.
            Node(
                package="ros_gz_sim",
                executable="create",
                name="robot_spawn_entity",
                arguments=[
                    "-name", robot_name,
                    "-allow_renaming", "true",
                    "-topic", "robot_description",
                    "-x", x_val,
                    "-y", y_val,
                    "-z", z_val,
                ],
                output="screen",
            ),

            # ROS-Gazebo Bridge
            # All topics in the bridge are now prefixed with the robot's name to ensure they are unique.
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ign_bridge",
                arguments=[
                    # Clock and TF don't need to be namespaced
                    "/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock",
                    "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                    # Namespaced topics
                    ['/model/', robot_name, '/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
                    ['/', robot_name, '_lidar/laser/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
                    ['/world/empty/model/', robot_name, '/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'],
                ],
                remappings=[
                    (['/world/empty/model/', robot_name, '/joint_state'], 'joint_states'),
                    (['/model/', robot_name, '/odometry'], 'true_odom'),
                ],
                output="screen",
            ),

            # Controller Manager Spawners
            # These will now operate within the robot's namespace.
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["robot01_base_controller"],
                output="screen",
            )
        ]
    )

    # --- LAUNCH DESCRIPTION ---
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        # Add the declared arguments to the LaunchDescription
        declare_robot_name_arg,
        declare_spawn_x_arg,
        declare_spawn_y_arg,
        declare_spawn_z_arg,
        # Add the GroupAction containing all the robot's nodes
        robot_nodes
    ])