# *********************** IMPORTS ***********************
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
# *******************************************************

# *********************** METHODE ***********************
def generate_launch_description():
    domain_id = SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='8')

    gazebo_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'],
    )

    catographer_launch = ExecuteProcess(
        cmd=['sleep', '15'],
        on_exit = [ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_cartographer', 'cartographer.launch.py', 'use_sim_time:=True'],
            )]
    )

    node_fernsteuerung = ExecuteProcess(
        cmd=['sleep', '20'],
        on_exit = [ExecuteProcess(
            cmd=['ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard'],
            output = 'screen',
            emulate_tty = True,
            )]
    )

    # LauchDescription speichern, alle Variablen da rein
    ld = LaunchDescription()
    ld.add_action(domain_id)
    ld.add_action(gazebo_launch)
    ld.add_action(catographer_launch)
    ld.add_action(node_fernsteuerung)
    # *********************

    # LaunchDescription zurückgeben, starte aller Variablen 
    return ld