import os
import launch
import launch_ros.actions
from launch.actions.execute_process import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package='robu',
            executable='myparameter',
            name='custom_minimal_Param_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'my_parameter':0.2}]
        )
    ])