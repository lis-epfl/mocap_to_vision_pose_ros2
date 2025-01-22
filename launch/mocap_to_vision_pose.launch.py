from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('mocap_to_vision_pose_ros2'),
        'config',
        'config.yaml'
    )
    mocap_to_vision_pose_node = Node(
        package='mocap_to_vision_pose_ros2',
        executable='mocap_to_vision_pose_node',
        name='mocap_to_vision_pose_node',
        parameters=[config],
        # prefix=['xterm -fa default -fs 10 -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True
    )

    ld.add_action(mocap_to_vision_pose_node)
    return ld
