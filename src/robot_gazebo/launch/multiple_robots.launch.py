import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    package_description = "robot_gazebo" 
    num_robots = 11
    delay_between_spawns = 5.0  # seconds
    
    single_robot_launch_file = 'one_robot.launch.py'
    
    pkg_share = get_package_share_directory(package_description)
    spawn_robot_launch_path = os.path.join(
        pkg_share, 'launch', single_robot_launch_file
    )

    ld = LaunchDescription()

    # Use a loop to create launch actions for each robot
    for i in range(num_robots):
        robot_name = f"robot0{i}"

        if i == 10:
            robot_name = "robot10"

        position_x = "-10.0"
        position_y = str(float(i * 2.0) - ((num_robots-1))) 
        position_z = "0.5"

        # Create an action to include the single robot launch file
        spawn_action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_robot_launch_path),
            launch_arguments={
                'robot_name': robot_name,
                'x': position_x,
                'y': position_y,
                'z': position_z,
            }.items()
        )
        
        # Use a TimerAction to delay the execution of the spawn_action
        timed_spawn_action = TimerAction(
            period=float(i*10),
            actions=[spawn_action]
        )
        
        # Add the timed action to the launch description
        ld.add_action(timed_spawn_action)

    return ld
