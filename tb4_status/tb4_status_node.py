from irobot_create_msgs.msg import DockStatus
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class TB4StatusNode(Node):
    """
    TurtleBot 4 Status Node (Monitors status of turtlebot 4 and publishes to database)
    This node subscribes to turtlebot4 topics to get all necessary information needed. This includes:
    - Battery State
    - Odometry (for speed and position)
    - Dock Status (charging state)
    - IP Address
    - etc.
    """
    def __init__(self):
        super().__init__('tb4_status_node')

        self.declare_parameter('robot_name', 'default_robot')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.get_logger().info(f"Starting TB4 Status Node for {self.robot_name}")

        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_callback,
            qos_profile_sensor_data 
        )

        self.declare_parameter('odom_rate', 2.0) # Hz
        self.odom_rate = self.get_parameter('odom_rate').get_parameter_value().double_value
        self.latest_odom_msg = None

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile_sensor_data 
        )

        self.processing_odom_time = self.create_timer(
            1.0 / self.odom_rate,
            self.process_speed
        )

    def battery_state_callback(self, msg: BatteryState):
        battery_percentage = round(msg.percentage * 100, 2)
        self.get_logger().info(
            f"Battery State: {battery_percentage}%"
        )

    def odom_callback(self, msg: Odometry):
        self.latest_odom_msg = msg

    def process_speed(self):
        if not self.latest_odom_msg:
            return

        linear_velocity = self.latest_odom_msg.twist.twist.linear
        vx = linear_velocity.x
        vy = linear_velocity.y
        vz = linear_velocity.z
        linear_speed = math.sqrt(vx**2 + vy**2 + vz**2)

        angular_velocity = self.latest_odom_msg.twist.twist.angular
        wz = angular_velocity.z

        self.get_logger().info(
            f"Linear Speed: {linear_speed:.2f} m/s, "
            f"Angular Velocity: {wz:.2f} rad/s"
        )



def main(args=None):
    rclpy.init(args=args)
    node = TB4StatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
