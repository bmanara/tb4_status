# tb4_status package
I'm only going to describe the nodes and launch files that are important/used. This includes:
- launch/single_robot.launch.py
- tb4_status/tb4_api_node.py
- tb4_status/tb4_ws_node.py

This package launches two ROS2 nodes that keep track of the robot states and allows the robot to receive tasks.
1. tb4_api_node: API endpoints to receive tasks and send to turtlebot4
2. tb4_ws_node: Keep track of turtlebot4 status and send regular updates through websockets

## User Guide
### API Endpoints (http://<turtlebot_ip>:8000) 
- POST `/dock`: Docking instruction for Turtlebot4
- POST `/undock`: Undocking instruction for Turtlebot4
- POST `/initialize`: Pass in position (x, y, z) and orientation (quaternion) for initial position _Deprecated_
- POST `/goal`: Pass in position (x, y, z) and orientation (quaternion) for goal position _Deprecated_
- POST `/trajectory`: Pass in a list of points as trajectory, translates points to poses for NavigateThroughPoses to plan
- POST `/pause`: Pause current trajectory. (Disables Nav2 lifecycle node, keeps track of remaining trajectory)
- POST `/resume`: Resume previous trajectory. (Enables Nav2 lifecycle node, takes previously saved trajectory)
- POST `/abort`: Abort current trajectory. (Disables Nav2 lifecycle node, clears trajectory, enable Nav2 lifecycle node)

### How to use
Used in tandem with the `my_tb4_nav` package.
```
cd turtlebot_ws
colcon build --packages-select tb4_status
source install/local_setup.bash
ros2 launch tb4_status single_robot.launch.py
```


## Developer's Guide
### tb4_api_node 
The API node primary purpose is to accept tasks through its API endpoints and translate the tasks into actions for the Turtlebot4 to perform.
Refer to API Endpoints section to see the supported tasks.

The node acts as a client, sending actions to goal servers set up by Nav2.

The implementations below uses clients/servers with ROS2 actions to send command to the turtlebot4:
- Docking & Undocking
- Trajectory / Publishing Nav2 Goal

All ROS2 actions have 3 methods that deal with the logic/implementation.
1. `send` method
    * This method is used to send the action to the server
    * Waits for server to start up before attempting to send action
2. `response` callback
    * Waits for response/feedback from server and logs them
3. `result` callback
    * Waits for result from server and logs them

The node will also update `tb4_ws_node` on any changes to the robot state it needs to know.

#### Private and Public Methods
| Method | Type | Description |
| --- | --- | --- |
| `send_dock_goal()` | Public | Sends a goal to the dock action server. |
| `send_undock_goal()` | Public | Sends a goal to the undock action server. |
| `set_initial_pose()` | Public | Publishes the initial pose of the robot to `amcl`. |
| `send_goal()` | Public | **Deprecated.** Sends a single pose goal to the `navigate_to_pose` action server. |
| `send_trajectory()` | Public | Sends a list of waypoints to the `navigate_through_poses` action server. |
| `send_pause()` | Public | Pauses the current navigation by deactivating Nav2 lifecycle nodes and saves the remaining trajectory. |
| `send_resume()` | Public | Resumes navigation by activating Nav2 lifecycle nodes and resending the saved trajectory. |
| `send_abort()` | Public | Aborts the current navigation, clears the trajectory, and resets Nav2. |
| `__dock_response_callback()` / `__dock_result_callback()` | Private | Callbacks for the dock action. |
| `__undock_response_callback()` / `__undock_result_callback()` | Private | Callbacks for the undock action. |
| `__goal_response_callback()` / `__goal_result_callback()` | Private | **Deprecated.** Callbacks for the `navigate_to_pose` action. |
| `__trajectory_feedback_callback()` | Private | Callback for receiving feedback during trajectory execution. |
| `__trajectory_response_callback()` / `__trajectory_result_callback()` | Private | Callbacks for handling the response and result of the trajectory action. |
| `__create_trajectory_message()` | Private | Creates a `NavigateThroughPoses.Goal` message from a list of trajectory points. |
| `__create_pose_message()` | Private | Creates a `PoseStamped` message for a single point in the trajectory. |
| `__read_map_from_json()` | Private | Reads the map landmarks from a JSON file. |
| `__temporary_abort()` | Private | Aborts a previous navigation task when a new trajectory is received. |
| `__update_status()` | Private | Publishes a status update message. |

### tb4_ws_node
The WebSocket node is responsible for maintaining a connection with a WebSocket server, tracking the robot's status, and sending periodic updates.

It subscribes to various ROS2 topics to gather information about the robot's battery, pose, and odometry. This information is then packaged and sent to the WebSocket server.

It sends a message to the server every second.

#### Private and Public Methods
| Method | Type | Description |
| --- | --- | --- |
| `battery_state_callback()` | Public | Callback for the `/battery_state` topic to update the robot's battery percentage. |
| `amcl_pose_callback()` | Public | Callback for the `/amcl_pose` topic to update the robot's current location. |
| `odom_callback()` | Public | Callback for the `/odom` topic to get odometry data for speed calculation. |
| `status_update_callback()` | Public | Callback for the `/status_update` topic to update the robot's status and task ID. |
| `calculate_speed()` | Public | Calculates the robot's current linear speed from odometry data. |
| `status_update_loop()` | Public (async) | A loop that periodically sends status updates to the WebSocket server. |
| `schedule_websocket_message()` | Public | Schedules a message to be sent through the WebSocket. |
| `send_websocket_message()` | Public (async) | Sends a JSON message to the WebSocket server. |
| `websocket_client()` | Public (async) | Manages the WebSocket client connection, handles reconnection, and runs the status update loop. |