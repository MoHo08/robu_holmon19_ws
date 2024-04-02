# *********************** IMPORTS ***********************
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
# *******************************************************

# *********************** KLASSE ************************
def generate_launch_description():
    domain_id = SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='8')


    node_gazebo = Node(
        package='turtlebot3_gazebo',                
        executable='turtlebot3_world.launch.py',
        output='screen'
    )

    node_catographer = Node(
        package='turtlebot3_cartographer',
        executable='cartographer.launch.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    node_fernsteuerung = Node(
        package='turtlebot3_teleop',
        executable='teleop_keyboard',
        output='screen',
        emulate_tty = True,
    )

    # LauchDescription speichern, alle Variablen da rein
    ld = LaunchDescription()
    ld.add_action(domain_id)
    ld.add_action(node_gazebo)
    ld.add_action(node_catographer)
    ld.add_action(node_fernsteuerung)
    # *********************

    # LaunchDescription zurückgeben, starte aller Variablen 
    return ld