import os
import launch
import launch_ros.actions
from launch.actions import ExecuteProcess

def generate_launch_description():

    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='8'),


        launch_ros.actions.Node(
            package='robu',
            executable='ex12_fibonacci_server',
            name='ex12_fibonacci_server',
            output='screen',
        ),

        launch_ros.actions.Node(
            package='robu',
            executable='ex13_fibonacci_client',
            name='ex13_fibonacci_client',
            output='screen',
        ),

        ExecuteProcess(
            cmd=['sleep', '5'],
            output = 'screen',
            name = 'sleep'
        ),

        ExecuteProcess(
            cmd=['ros2', 'action', 'send_goal', '/fibonacci', 'robu_interfaces/action/Fibonacci', '{order: 5}'],
            output = 'screen',
            name = 'call_fibonacci_action'
        )
    ])