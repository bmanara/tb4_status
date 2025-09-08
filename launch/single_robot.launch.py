from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    ld = LaunchDescription()

    namespace = 'tb6'

    robot_status_node = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            Node(
                package='tb4_status',
                executable='tb4_api_node',
                name='turtlebot_api_node',
                output='screen',
                parameters=[{'robot_name': namespace}],
            )
        ]
    )

    robot_ws_node = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            Node(
                package='tb4_status',
                executable='tb4_ws_node',
                name='turtlebot_ws_node',
                output='screen',
                parameters=[{'robot_name': namespace}],
            )
        ]
    )

    ld.add_action(robot_status_node)
    ld.add_action(robot_ws_node)

    return ld