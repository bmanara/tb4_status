# tb4_status package
I'm only going to describe the nodes and launch files that are important/used. This includes:
- launch/single_robot.launch.py
- tb4_status/tb4_api_node.py

This package launches a ros2 node that allows devices connected to the same network as the turtlebot4 to interact with its endpoints.

## User Guide
### Endpoints (http://<turtlebot_ip>:8000) 
- GET `/status`: No arguments
- POST `/dock`: No arguments
- POST `/undock`: No arguments
- POST `/initialize`: Pass in position (x, y, z) and orientation (quaternion)
- POST `/trajectory`: Pass in a list of points as trajectory

### How to use
```
cd turtlebot_ws
colcon build --packages-select tb4_status
source install/local_setup.bash
ros2 launch tb4_status single_robot.launch.py
```

## Developer's Guide
### ROS2 Topics
The implementations below uses publishers/subscribers with ROS2 topics to send commands to the turtlebot4:
- Publishers
    - Initialization of initial position (requires localization to be active)
- Subscribers
    - Robot status (Battery)



### ROS2 Actions
The implementations below uses clients/servers with ROS2 actions to send command to the turtlebot4:
- Docking & Undocking
- Trajectory / Publishing Nav2 Goal (implementation WILL change to suit project needs)

All ROS2 actions have 3 methods that deal with the logic/implementation.
1. `send` method
    * This method is used to send the action to the server
    * Waits for server to start up before attempting to send action
2. `response` callback
    * Waits for response/feedback from server and logs them
3. `result` callback
    * Waits for result from server and logs them
