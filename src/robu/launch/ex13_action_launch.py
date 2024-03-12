# *********************** IMPORTS ***********************
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
# *******************************************************

# *********************** KLASSE ************************
def generate_launch_description():

    # ***** Variablen *****
    mypackage = get_package_share_directory('robu')
    domain_id = SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='8')

    # Node in Variable speichern mit notwendigen Parameter
    node_action_server = Node(
        package='robu',                     # Pkg, wo Node drinnen
        executable='ex12_fibonacci_server', # Node, Name laut Setup.py
        output='screen',                    # optional, Ausgabe im CMD
        emulate_tty = True,                 # optional, Tastatur etc. kann verwendet werden
    )
    # Node in Variable speichern mit notwendigen Parameter
    node_action_client = Node(
        package='robu',
        executable='ex13_fibonacci_client',
        output='screen',
        emulate_tty = True,
    )
    # CLI-Befehle in Variable speichern
    exec_action = ExecuteProcess(
        cmd=['sleep', '5'],                 # cmd Befehl, warte 5 sec
        output = 'screen',                  # optional, Ausgabe im CMD                    
        on_exit = [ExecuteProcess(          # Am Ende (nach 5s) wird Aufgerufen
            cmd=['ros2', 'action', 'send_goal', '--feedback', 'fibonacci', 'robu_interfaces/action/Fibonacci', '{order: 5}'],
            output = 'screen',
            name = 'call_fibonacci_action'
            )]
    )
    # LauchDescription speichern, alle Variablen da rein
    ld = LaunchDescription()
    ld.add_action(domain_id)
    ld.add_action(node_action_server)
    ld.add_action(node_action_client)
    ld.add_action(exec_action)
    # *********************

    # LaunchDescription zurückgeben, starte aller Variablen 
    return ld