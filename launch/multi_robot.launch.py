from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


ROBOT_NAMES = [
    'tb7',
    'tb15'
]

def generate_launch_description():
    ld = LaunchDescription()

    for name in ROBOT_NAMES:
        robot_status_node = GroupAction(
            actions=[
                PushRosNamespace(name),
                Node(
                    package='tb4_status',
                    executable='tb4_status_node',
                    name='tb4_status_node',
                    output='screen',
                    parameters=[{'robot_name': name}],
                )
            ]
        )

        ld.add_action(robot_status_node)

    return ld
