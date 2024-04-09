# CLI-Commands
#1. action list: ros2 action list
#2. action info: ros2 action info <action info> -t
#3. interface list: ros2 interface show /nav2_msgs/action/NavigateToPose
#4. action call: ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.75, y: 1.0, z: 0.01}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" 
