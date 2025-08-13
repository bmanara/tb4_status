from irobot_create_msgs.msg import DockStatus

from sensor_msgs.msg import BatteryState

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class TB4StatusNode(Node):
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

    def battery_state_callback(self, msg: BatteryState):
        battery_percentage = round(msg.percentage * 100, 2)
        self.get_logger().info(
            f"Battery State ({self.robot_name}): {battery_percentage}%"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TB4StatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
