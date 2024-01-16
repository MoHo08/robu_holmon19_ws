from launch import LaunchDescription
from launch_ros.actions import Node

def gernerate_launch_description():
    return LaunchDescription([
        Node(
            package='robu',
            executable='myparameter',
            name='custom_minimal_Param_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'my_parameter' : 'saturn'}
            ]
        )
    ])