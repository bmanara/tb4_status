import asyncio
import json
import math
import threading
import websockets

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tb4_status_interface.msg import StatusUpdate


class TB4WsNode(Node):
    """
    TurtleBot 4 WebSocket Node
    This node provides a WebSocket server to receive updates from the TurtleBot 4 robot.
    It allows clients to connect and receive real-time updates on the robot's status.
    This node will also keep track of the robot's status.
    """
    def __init__(self):
        super().__init__('tb4_ws_node')
        self.loop = None

        self.request_id = 0 
        self.task_status = 0

        self.ip_address = '10.100.111.90'
        self.uri = f"ws://10.100.111.76:5100/ws"
        self.websocket = None
        self.status = 'ready'
        self.latest_odom_msg = None

        # State of robot
        self.battery_percent = None
        self.current_location_x = 0.0
        self.current_location_y = 0.0
        self.current_task_id = None

        self.map_name = 'level5_arena_v3'

        # Subscribers
        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_callback,
            qos_profile_sensor_data
        )

        self.amcl_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_pose_callback,
            10
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.status_update_subscriber = self.create_subscription(
            StatusUpdate,
            'status_update',
            self.status_update_callback,
            10
        )

    def battery_state_callback(self, msg: BatteryState):
        """
        Callback for battery state updates.
        """
        self.battery_percent = msg.percentage * 100 if msg.percentage is not None else 0.0

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback for AMCL pose updates.
        """
        self.current_location_x = msg.pose.pose.position.x
        self.current_location_y = msg.pose.pose.position.y

    def odom_callback(self, msg: Odometry):
        """
        Callback for odometry updates.
        """
        self.latest_odom_msg = msg

    def status_update_callback(self, msg: StatusUpdate):
        """
        Callback for status update messages.
        """
        status = msg.status
        if status == 0:
            self.status = 'idle'
        elif status == 1:
            self.status = 'executing'
        elif status == 2:
            self.status = 'charging'
        
        task_id = msg.task_id
        self.current_task_id = task_id if task_id != -1 else None
        self.request_id = msg.request_id
        self.task_status = msg.task_status

    def calculate_speed(self):
        """
        Calculate the current speed of the robot from the latest odometry message.
        """
        if not self.latest_odom_msg:
            return 0.0

        linear_velocity = self.latest_odom_msg.twist.twist.linear
        vx = linear_velocity.x
        vy = linear_velocity.y
        vz = linear_velocity.z
        linear_speed = math.sqrt(vx**2 + vy**2 + vz**2)

        return linear_speed

    async def robot_status_update_loop(self):
        """
        Periodically send robot status updates to connected WebSocket clients.
        """
        while rclpy.ok():
            current_location = {
                'x': self.current_location_x,
                'y': self.current_location_y
            }

            curr_speed = self.calculate_speed()
            payload = {
                'auc_id': self.get_namespace().strip('/'),
                'battery_percent': self.battery_percent,
                'ip_address': self.ip_address,
                'location': current_location,
                'status': self.status,
                'task_id': self.current_task_id,
                'timestamp': self.get_clock().now().to_msg().sec,
                'waypoints': None,
                'curr_speed': curr_speed,
                'map_name': self.map_name,
                'speed_limit': 5.0
            }

            data = {
                'header': {
                    'status_id': 5,
                    'type': 2
                },
                'payload': payload
            }

            self.schedule_websocket_message(data)
            await asyncio.sleep(1)  # Send update every 1 second

    async def task_status_update_loop(self):
        """
        Periodically send task status updates to connected WebSocket clients.
        Provides mock data for demonstration purposes.
        0. header->requestid
        1. payload->data->robot_mission_data->mission_status->mission_id
        2. payload->data->robot_mission_data->curr_sub_mission_status->mission_custom_data
        3. payload->data->robot_mission_data->curr_sub_mission_status->tasks_summary (supposed to be a list of tasks, but...)
        
        """
        while rclpy.ok():
            payload = {
                'auc_id': self.get_namespace().strip('/'),
                "data": [
                    {
                        "robot_mission_data": {
                            "curr_sub_mission_status": {
                                "mission_custom_data": "[{\"data\":{\"request_str\":\"20250709_091704_24\",\"requestid\":" + str(self.request_id) + "},\"name\":\"i2rfm\"}]",
                                "tasks_summary": "[{\"state\":" + str(self.task_status) + ",\"type\":-3,\"uid\":3}]"
                            },
                            "mission_status": {
                                "mission_id": self.current_task_id if self.current_task_id is not None else -1
                            }
                        }
                    }
                ]
            }

            data = {
                'header': {
                    'status_id': 4,
                    'type': 2,
                    'requestid': self.request_id
                },
                'payload': payload
            }

            self.schedule_websocket_message(data)
            await asyncio.sleep(1)  # Send update every 1 second


    def schedule_websocket_message(self, data):
        if self.loop and not self.loop.is_closed():
            asyncio.run_coroutine_threadsafe(self.send_websocket_message(data), self.loop)

    async def send_websocket_message(self, data):
        if self.websocket: 
            try:
                message = json.dumps(data)
                await self.websocket.send(message)
            except Exception as e:
                self.get_logger().error(f"Error sending WebSocket message: {e}")
    
    async def websocket_client(self):
        self.loop = asyncio.get_event_loop()

        # Start the status update loop as a background task
        robot_status_loop = asyncio.create_task(self.robot_status_update_loop())

        task_status_loop = asyncio.create_task(self.task_status_update_loop())

        while rclpy.ok():
            try:
                self.get_logger().info(f'Connecting to Websocket server @ {self.uri}')
                async with websockets.connect(self.uri) as websocket:
                    self.websocket = websocket
                    self.get_logger().info('WebSocket connection established.')

                    auth_payload = {
                        'header': {
                            'clienttype': 0
                        }
                    }
                    await self.send_websocket_message(auth_payload)

                    async for message in websocket:
                        self.get_logger().info(f'{message}')
            except websockets.ConnectionClosed:
                self.get_logger().warning('WebSocket connection closed, retrying in 5 seconds...')
                self.websocket = None
                await asyncio.sleep(5)
            except Exception as e:
                self.get_logger().error(f'WebSocket error: {e}, retrying in 5 seconds...')
                self.websocket = None
                await asyncio.sleep(5)
        
        # Cancel the status task when exiting
        robot_status_loop.cancel()
        task_status_loop.cancel()

    def ros_spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    node = TB4WsNode()

    ros_thread = threading.Thread(target=node.ros_spin, daemon=True)
    ros_thread.start()

    try:
        asyncio.run(node.websocket_client())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()