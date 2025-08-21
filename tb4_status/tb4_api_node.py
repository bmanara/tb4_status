import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Header
from sensor_msgs.msg import BatteryState
from irobot_create_msgs.action import Dock, Undock
from geometry_msgs.msg import PoseWithCovarianceStamped

import math
import threading
from fastapi import FastAPI
import uvicorn
from pydantic import BaseModel


# Constants
DEFAULT_INITIAL_POSE_COVARIANCE = [0.0] * 36
DEFAULT_INITIAL_POSE_COVARIANCE[0] = 0.25
DEFAULT_INITIAL_POSE_COVARIANCE[7] = 0.25
DEFAULT_INITIAL_POSE_COVARIANCE[35] = 0.06853891945200942


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
        self._dock_client = ActionClient(self, Dock, 'dock')
        self._undock_client = ActionClient(self, Undock, 'undock')

        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            10
        )

        self.current_battery_state = None
        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_callback,
            qos_profile_sensor_data
        )

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

    def send_trajectory(self, trajectory: list):
        """
        Handles translation and sending a trajectory command to the TurtleBot 4.
        """
        if trajectory is None or len(trajectory) == 0:
            self.get_logger().error("Received empty trajectory command.")
            return

        nav2_trajectory_msg = self.__create_trajectory_message(trajectory)
        
        return NotImplementedError("Trajectory command handling is not implemented yet.")

    def __create_trajectory_message(self, trajectory: list):
        return NotImplementedError("Trajectory message creation is not implemented yet.")


class PoseModel(BaseModel):
    position: tuple
    orientation: tuple

class TrajectoryModel(BaseModel):
    trajectory: list

app = FastAPI()

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


# API Endpoints
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

@app.post('/trajectory')
async def trajectory(trajectory_model: TrajectoryModel):
    trajectory = trajectory_model.trajectory
    tb4_api_node.send_trajectory(trajectory)
    return {"message": "Trajectory command received", "trajectory": trajectory}


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
