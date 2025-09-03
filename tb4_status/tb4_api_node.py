import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import BatteryState
from tb4_status_interface.msg import StatusUpdate

from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from irobot_create_msgs.action import Dock, Undock

import json
import threading
from fastapi import FastAPI
import uvicorn
from pydantic import BaseModel


# Constants
DEFAULT_INITIAL_POSE_COVARIANCE = [0.0] * 36
DEFAULT_INITIAL_POSE_COVARIANCE[0] = 0.25
DEFAULT_INITIAL_POSE_COVARIANCE[7] = 0.25
DEFAULT_INITIAL_POSE_COVARIANCE[35] = 0.06853891945200942

# DUMMY_POINTS = [
#     (-12.0, -2.0, 0.0),
#     (-11.0, -2.0, 0.0),
#     (-10.0, -2.0, 0.0),
#     (-9.0, -2.0, 0.0),
#     (-8.0, -2.0, 0.0),
#     (-7.0, -2.0, 0.0),
#     (-6.0, -2.0, 0.0),
#     (-12.0, -1.0, 0.0),
#     (-11.0, -1.0, 0.0),
#     (-10.0, -1.0, 0.0),
#     (-9.0, -1.0, 0.0),
#     (-8.0, -1.0, 0.0),
#     (-7.0, -1.0, 0.0),
#     (-6.0, -1.0, 0.0),
#     (-12.0, 0.0, 0.0),
#     (-11.0, 0.0, 0.0),
#     (-10.0, 0.0, 0.0),
#     (-9.0, 0.0, 0.0),
#     (-8.0, 0.0, 0.0),
#     (-7.0, 0.0, 0.0),
#     (-6.0, 0.0, 0.0),
# ]


class TB4ApiNode(Node):
    """
    TurtleBot 4 API Node (Provides REST API for controlling TurtleBot 4)
    This node provides a REST API to control the TurtleBot 4 robot.
    Provided Functionality:
    - Docking and undocking the robot
    - Getting robot status (battery state, TODO: other states)
    - Receiving and translating trajectory commands (TODO)
    """
    def __init__(self):
        super().__init__('tb4_api_node')
        self.map_landmarks = None
        self._dock_client = ActionClient(self, Dock, 'dock')
        self._undock_client = ActionClient(self, Undock, 'undock')

        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            10
        )

        self.status_update_publisher = self.create_publisher(
            StatusUpdate,
            'status_update',
            10
        )

        self.current_battery_state = None
        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_callback,
            qos_profile_sensor_data
        )

        self._navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._navigate_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.map_landmarks = self.__read_map_from_json()
        self.get_logger().info(f"Map landmarks loaded: {self.map_landmarks}")

    def battery_state_callback(self, msg: BatteryState):
        """
        Callback for battery state updates.
        """
        self.current_battery_state = msg

    def send_dock_goal(self):
        """
        Handles sending a dock goal to the TurtleBot 4.
        """
        dock_goal_msg = Dock.Goal()
        self._dock_client.wait_for_server()

        self.get_logger().info("Sending dock goal to create3...")

        future = self._dock_client.send_goal_async(dock_goal_msg)
        future.add_done_callback(self.__dock_response_callback)

    def __dock_response_callback(self, future):
        """
        Private callback for the dock response.
        """
        dock_goal_handle = future.result()
        if not dock_goal_handle.accepted:
            self.get_logger().error("Dock goal was rejected.")
            return

        self.get_logger().info("Dock goal accepted, waiting for result...")
        result_future = dock_goal_handle.get_result_async()
        result_future.add_done_callback(self.__dock_result_callback)

    def __dock_result_callback(self, future):
        """
        Private callback for the dock result.
        """
        result = future.result().result
        if result is None:
            self.get_logger().error("Dock action failed.")
        else:
            self.get_logger().info(f"Dock action completed with result: {result}")

    def send_undock_goal(self):
        """
        Handles sending an undock goal to the TurtleBot 4.
        """
        undock_goal_msg = Undock.Goal()
        self._undock_client.wait_for_server()
        
        self.get_logger().info("Sending undock goal to create3...")

        future = self._undock_client.send_goal_async(undock_goal_msg)
        future.add_done_callback(self.__undock_response_callback)

    def __undock_response_callback(self, future):
        """
        Private callback for the undock response.
        """
        undock_goal_handle = future.result()
        if not undock_goal_handle.accepted:
            self.get_logger().error("Undock goal was rejected.")
            return
            
        self.get_logger().info("Undock goal accepted, waiting for result...")
        result_future = undock_goal_handle.get_result_async()
        result_future.add_done_callback(self.__undock_result_callback)

    def __undock_result_callback(self, future):
        """
        Private callback for the undock result.
        """
        result = future.result().result
        if result is None:
            self.get_logger().error("Undock action failed.")
        else:
            self.get_logger().info(f"Undock action completed with result: {result}")

    def set_initial_pose(self, position: tuple = (0.0, 0.0, 0.0), orientation: tuple = (0.0, 0.0, 0.0, 1.0)):
        """
        Sets the initial pose of the TurtleBot 4.
        """
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.pose.position.x = position[0]
        pose_msg.pose.pose.position.y = position[1]
        pose_msg.pose.pose.position.z = position[2]

        pose_msg.pose.pose.orientation.x = orientation[0]
        pose_msg.pose.pose.orientation.y = orientation[1]
        pose_msg.pose.pose.orientation.z = orientation[2]
        pose_msg.pose.pose.orientation.w = orientation[3]

        pose_msg.pose.covariance = DEFAULT_INITIAL_POSE_COVARIANCE

        self.initial_pose_publisher.publish(pose_msg)

        return True

    def send_goal(self, position: tuple, orientation: tuple):
        """
        Handles translation and sending a goal command to the TurtleBot 4.
        """
        if position is None or len(position) != 2:
            self.get_logger().error("Invalid position provided. Must be a tuple of (x, y).")
            return
        
        if orientation is None or len(orientation) != 4:
            self.get_logger().error("Invalid orientation provided. Must be a tuple of (x, y, z, w).")
            return

        nav2_goal_msg = NavigateToPose.Goal()

        nav2_goal_msg.pose.header = Header()
        nav2_goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        nav2_goal_msg.pose.header.frame_id = 'map'

        nav2_goal_msg.pose.pose.position.x = position[0]
        nav2_goal_msg.pose.pose.position.y = position[1]
        nav2_goal_msg.pose.pose.position.z = 0.0

        nav2_goal_msg.pose.pose.orientation.x = orientation[0]
        nav2_goal_msg.pose.pose.orientation.y = orientation[1]
        nav2_goal_msg.pose.pose.orientation.z = orientation[2]
        nav2_goal_msg.pose.pose.orientation.w = orientation[3]

        self._navigate_to_pose_client.wait_for_server()

        self.get_logger().info("Sending goal command to TurtleBot 4...")

        future = self._navigate_to_pose_client.send_goal_async(nav2_goal_msg)
        future.add_done_callback(self.__goal_response_callback)

    def __goal_response_callback(self, future):
        """
        Private callback for the goal response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected.")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.__goal_result_callback)

    def __goal_result_callback(self, future):
        """
        Private callback for the goal result.
        """
        result = future.result().result
        if result is None:
            self.get_logger().error("Goal action failed.")
        else:
            self.get_logger().info(f"Goal action completed with result: {result}")

    def send_trajectory(self, trajectory: list):
        trajectory_msg = self.__create_trajectory_message(trajectory)

    def __trajectory_response_callback(self, future):
        """
        Private callback for the trajectory response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory was rejected.")
            return

        self.get_logger().info("Trajectory accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.__trajectory_result_callback)

    def __trajectory_result_callback(self, future):
        """
        Private callback for the trajectory result.
        """
        result = future.result().result
        if result is None:
            self.get_logger().error("Trajectory action failed.")
        else:
            self.get_logger().info(f"Trajectory action completed with result: {result}")

    def __create_trajectory_message(self, trajectory: list):
        # For now, send a dummy trajectory message
        # TODO: This should be replaced with actual trajectory message creation logic
        poses = [self.__create_pose_message(traj) for traj in trajectory]
        trajectory_msg = NavigateThroughPoses.Goal()
        trajectory_msg.poses = poses
        
        self._navigate_through_poses_client.wait_for_server()
        self.get_logger().info("Sending trajectory command to TurtleBot 4...")

        # TODO: Probably best NOT to update status of turtlebot here...
        status_msg = StatusUpdate()
        status_msg.status = 1
        self.status_update_publisher.publish(status_msg)

        future = self._navigate_through_poses_client.send_goal_async(trajectory_msg)
        future.add_done_callback(self.__trajectory_response_callback)

    def __trajectory_response_callback(self, future):
        """
        Private callback for the trajectory response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory was rejected.")
            return

        self.get_logger().info("Trajectory accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.__trajectory_result_callback)

    def __goal_result_callback(self, future):
        """
        Private callback for the trajectory result.
        """
        result = future.result().result
        if result is None:
            self.get_logger().error("Trajectory action failed.")
        else:
            self.get_logger().info(f"Trajectory action completed with result: {result}")

    def __create_pose_message(self, p: int):
        point = self.map_landmarks[p]
        self.get_logger().debug(f"Creating pose message for landmark {p} at point {point}")

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = point[0] - 3.9
        pose_msg.pose.position.y = point[1] - 0.9
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        return pose_msg

    def __read_map_from_json(self):
        """
        Note that this method forces user to run program in turtlebot4_ws.
        Though, it is not the only thing forcing the above requirement...
        """
        file_path = '/home/ubuntu/turtlebot4_ws/level5_arena_v3-1.json'
        with open(file_path, 'r') as f:
            map_data = json.load(f)
            map_landmarks = {vertex["user_uid"]: (vertex["robot_pose_x"], vertex["robot_pose_y"]) for vertex in map_data["landmarks"]}

            return map_landmarks

class PoseModel(BaseModel):
    position: tuple
    orientation: tuple

class GoalModel(BaseModel):
    position: tuple
    orientation: tuple

class TrajectoryModel(BaseModel):
    trajectory: list

app = FastAPI()

# API Endpoints
@app.get('/status')
async def status():
    """
    Endpoint to get the status of the TurtleBot 4.
    """
    # Here you would typically gather status information from the robot
    # For now, we will return a placeholder response
    status = {}
    status['battery_percentage'] = tb4_api_node.current_battery_state.percentage * 100 if tb4_api_node.current_battery_state else None
    return {"status": status}

@app.post('/dock')
async def dock():
    success = tb4_api_node.send_dock_goal()
    return {"success": success}

@app.post('/undock')
async def undock():
    success = tb4_api_node.send_undock_goal()
    return {"success": success}

@app.post('/initialize')
async def initialize(pose_model: PoseModel):
    position = pose_model.position
    orientation = pose_model.orientation
    success = tb4_api_node.set_initial_pose(position, orientation)
    return {"success": success}

@app.post('/goal')
async def goal(goal_model: GoalModel):
    position = goal_model.position
    orientation = goal_model.orientation
    tb4_api_node.send_goal(position, orientation)
    return {"message": "Goal command received", "goal": goal_model}

@app.post('/trajectory')
async def trajectory(trajectory_model: TrajectoryModel):
    trajectory = trajectory_model.trajectory
    tb4_api_node.send_trajectory(trajectory)
    return {"message": "Trajectory command received", "trajectory": trajectory_model}


def main(args=None):
    rclpy.init()

    global tb4_api_node
    tb4_api_node = TB4ApiNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(tb4_api_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    uvicorn.run(app, host='0.0.0.0', port=8000)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
