# tb4_status package
I'm only going to describe the nodes and launch files that are important/used. This includes:
- launch/single_robot.launch.py
- tb4_status/tb4_api_node.py

This package launches a ros2 node that allows devices connected to the same network as the turtlebot4 to interact with its endpoints.

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