# CLI-Commands
#1. action list: ros2 action list
#2. action info: ros2 action info <action info> -t
#3. interface list: ros2 interface show /nav2_msgs/action/NavigateToPose
#4. action call: ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.75, y: 1.0, z: 0.01}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" 

# Phyton Programm zum navigieren
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String


class Nav2Pose(Node):
    def __init__(self)
        super().__init__('nav2_pose')
        self.pose_sub = self.create_subscription(String, '/nav2_pose', self.nav2_pose_callback, 10)

        self.declare_parameters('',
                                [
                                    ('pose_initial', [0.0, 0.0, 0.0, 1.0])  #x, y, z, w
                                ])