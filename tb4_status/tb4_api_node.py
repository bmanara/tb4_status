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

from nav2_msgs.srv import ManageLifecycleNodes

import json
import threading
from fastapi import FastAPI
from fastapi.responses import JSONResponse
import uvicorn
from pydantic import BaseModel
import time
from typing import Optional


# Constants
DEFAULT_INITIAL_POSE_POSITION = (-6.0, -1.0, 0.0)
DEFAULT_INITIAL_POSE_ORIENTATION = (0.0, 0.0, 0.999596054053229, 0.028420568629322587)

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
    - Receiving and translating trajectory commands
    - Sending status updates to websocket node regarding current status
    """
    def __init__(self):
        super().__init__('tb4_api_node')
        self.map_landmarks = None
        self.trajectory = None
        self.latest_trajectory_feedback_msg = None
        self.current_task_id = None

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

        self._navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._navigate_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.map_landmarks = self.__read_map_from_json()
        self.get_logger().info(f"Map landmarks loaded: {self.map_landmarks}")

        self.nav2_client = self.create_client(
            ManageLifecycleNodes,
            'lifecycle_manager_navigation/manage_nodes'
        )

        while not self.nav2_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Nav2 lifecycle service not available, waiting...')

        self.get_logger().info('Nav2 lifecycle service is available.')

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

        self.get_logger().info("Waiting for AMCL subscriber...")
        while self.initial_pose_publisher.get_subscription_count() == 0:
            if not rclpy.ok():
                return
            
            self.get_logger().info("Waiting for AMCL subscriber...")
            time.sleep(5.0)
        
        self.get_logger().info(f"Publishing initial pose: {pose_msg} to AMCL")
        self.initial_pose_publisher.publish(pose_msg)

        return True

    def send_goal(self, position: tuple, orientation: tuple):
        """
        Handles translation and sending a goal command to the TurtleBot 4. Deprecated, use send_trajectory instead.
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
        Private callback for the goal response. Deprecated, use send_goal's callback instead.
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
        Private callback for the goal result. Deprecated, use send_trajectory's callback instead.
        """
        result = future.result().result
        if result is None:
            self.get_logger().error("Goal action failed.")
        else:
            self.get_logger().info(f"Goal action completed with result: {result}")

    def send_trajectory(self, trajectory: list, task_id: int):
        previous_traj = self.trajectory
        trajectory_msg = self.__create_trajectory_message(trajectory)

        if previous_traj:
            # TODO: If there is a better way to deal with this logic, please update.
            self.__temporary_abort() # If new trajectory comes in, best to abort previous Nav task before sending new one
            time.sleep(5)

        self._navigate_through_poses_client.wait_for_server()
        future = self._navigate_through_poses_client.send_goal_async(trajectory_msg, self.__trajectory_feedback_callback)
        future.add_done_callback(self.__trajectory_response_callback)

        self.current_task_id = task_id

    def __trajectory_feedback_callback(self, feedback_msg):
        """
        Private callback for the trajectory feedback.
        Update self.trajectory to reflect remaining waypoints.
        """
        feedback = feedback_msg.feedback
        self.latest_trajectory_feedback_msg = feedback

    def __trajectory_response_callback(self, future):
        """
        Private callback for the trajectory response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory was rejected.")
            return

        self.__update_status(1, self.current_task_id)  # set to executing when trajectory is accepted

        self.get_logger().info("Trajectory accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.__trajectory_result_callback)

    def __trajectory_result_callback(self, future):
        """
        Private callback for the trajectory result.
        """
        result = future.result().result

        # Possible status codes:
        # 4 = SUCCEEDED (completed successfully)
        # 5 = ABORTED (user cancelled)
        # 6 = CANCELLED (lifecycle pause sends cancel request)
        status = future.result().status
        if result is None:
            self.get_logger().error("Trajectory action failed.")
        else:
            self.get_logger().info(f"Trajectory action completed with result: {result}")
            self.get_logger().info(f"Trajectory action completed with status: {status}")

        # Differentiate between SUCCEEDED, ABORTED, and CANCELLED for status update
        if status == 4:
            self.__update_status(0, None)  # set to idle when only when trajectory is completed (or failed)
            self.trajectory = None

    def __create_trajectory_message(self, trajectory: list):
        self.trajectory = trajectory
        poses = [self.__create_pose_message(traj) for traj in trajectory]
        trajectory_msg = NavigateThroughPoses.Goal()
        trajectory_msg.poses = poses

        return trajectory_msg

    def __create_pose_message(self, p: int):
        point = self.map_landmarks[p]
        self.get_logger().debug(f"Creating pose message for landmark {p} at point {point}")

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = point[0]
        pose_msg.pose.position.y = point[1]
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

    def send_pause(self):
        """
        Send a request to Nav2 lifecycle to deactivate the navigation stack.
        Saves the remaining trajectory if there is one.
        """
        pause_request = ManageLifecycleNodes.Request()
        pause_request.command = ManageLifecycleNodes.Request.PAUSE

        # Update self.trajectory to only include remaining waypoints
        if self.latest_trajectory_feedback_msg:
            remaining_points = self.latest_trajectory_feedback_msg.number_of_poses_remaining
            if len(self.trajectory) > remaining_points:
                self.trajectory = self.trajectory[-remaining_points:]
            elif len(self.trajectory) == None:
                self.trajectory = None

        return self.nav2_client.call_async(pause_request)

    def send_resume(self):
        """
        Send a request to Nav2 lifecycle to activate the navigation stack.
        Will send the remaining trajectory if there was one before pausing.
        """
        resume_request = ManageLifecycleNodes.Request()
        resume_request.command = ManageLifecycleNodes.Request.RESUME

        self.nav2_client.call_async(resume_request)
        if self.trajectory:
            # If there was a trajectory before pausing, resend it once action server is available
            while not self._navigate_through_poses_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().info('NavigateThroughPoses action server not available, waiting...')

            time.sleep(5.0) # Give some extra time for the server to be fully ready
            self.get_logger().info('NavigateThroughPoses action server is available, resending trajectory...')
            self.send_trajectory(self.trajectory, self.current_task_id)

    def send_abort(self):
        """
        Send a request to Nav2 lifecycle to deactivate the navigation stack.
        Clears any saved trajectory and set robot status to idle.
        """
        pause_request = ManageLifecycleNodes.Request()
        pause_request.command = ManageLifecycleNodes.Request.PAUSE

        self.nav2_client.call_async(pause_request)

        self.trajectory = None
        time.sleep(2)

        self.send_resume() 
        self.__update_status(0, None)  # set to idle when aborting task

    def __temporary_abort(self):
        """
        Private method to abort previous Nav task when new trajectory is given while running
        """
        pause_request = ManageLifecycleNodes.Request()
        pause_request.command = ManageLifecycleNodes.Request.PAUSE
        self.nav2_client.call_async(pause_request)

        resume_request = ManageLifecycleNodes.Request()
        resume_request.command = ManageLifecycleNodes.Request.RESUME
        self.nav2_client.call_async(resume_request)

    def __update_status(self, status: int, task_id: Optional[int]):
        """
        Update the robot status and publish it.
        Status codes:
        0 - idle
        1 - executing
        2 - paused
        3 - error
        task_id can be None if no task is being executed
        """
        status_msg = StatusUpdate()
        status_msg.status = status
        status_msg.task_id = task_id if task_id else -1
        self.status_update_publisher.publish(status_msg)

        self.current_task_id = task_id

class PoseModel(BaseModel):
    position: tuple
    orientation: tuple

class GoalModel(BaseModel):
    position: tuple
    orientation: tuple

class TrajectoryModel(BaseModel):
    trajectory: list
    task_id: int

app = FastAPI()

# API Endpoints
@app.post('/dock')
async def dock():
    success = tb4_api_node.send_dock_goal()
    return {"success": success}

@app.post('/undock')
async def undock():
    success = tb4_api_node.send_undock_goal()
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
    task_id = trajectory_model.task_id
    tb4_api_node.send_trajectory(trajectory, task_id)
    return {"message": "Trajectory command received", "trajectory": trajectory_model}

@app.post('/pause')
async def pause():
    paused_task_id = tb4_api_node.current_task_id
    tb4_api_node.send_pause()
    return {"message": "Pause command received", "task_id": paused_task_id}

@app.post('/resume')
async def resume():
    tb4_api_node.send_resume()
    return {"message": "Resume command received"}

@app.post('/abort')
async def abort():
    aborted_task_id = tb4_api_node.current_task_id
    tb4_api_node.send_abort()
    return {"message": "Abort command received", "task_id": aborted_task_id}


def main(args=None):
    rclpy.init()

    global tb4_api_node
    tb4_api_node = TB4ApiNode()
    tb4_api_node.set_initial_pose(DEFAULT_INITIAL_POSE_POSITION, DEFAULT_INITIAL_POSE_ORIENTATION)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(tb4_api_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    uvicorn.run(app, host='0.0.0.0', port=8000)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
