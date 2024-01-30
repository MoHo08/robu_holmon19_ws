import launch
import launch_ros.actions

def generate_launch_description():

    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package='robu',
            executable='myparameter',
            name='custom_minimal_Param_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'my_parameter':0.2},
                        {'forward_speed_wf_slow':0.1124}]
        )
    ])