from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # declare a launch argument for namespace
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node'
    )

    topic_name = DeclareLaunchArgument(
        'topic_name',
        default_value='/mocap_to_vision_pose',
        description='Topic name for the mocap to vision pose node'
    )

    topic_name_str = LaunchConfiguration('topic_name')

    # get config file path
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('mocap_to_vision_pose_ros2'),
        'config',
        'config.yaml'
    )

    # create node with namespace from launch argument
    mocap_to_vision_pose_node = Node(
        package='mocap_to_vision_pose_ros2',
        executable='mocap_to_vision_pose_node',
        namespace=LaunchConfiguration('namespace'),
        name='mocap_to_vision_pose_node',
        parameters=[config, {'topic_name': topic_name_str}],
        # prefix=['xterm -fa default -fs 10 -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True
    )

    # add actions
    ld.add_action(namespace_arg)
    ld.add_action(topic_name)
    ld.add_action(mocap_to_vision_pose_node)

    return ld
