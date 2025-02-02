#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from launch import LaunchService
from launch import LaunchDescription
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import time
import subprocess
from mavros_msgs.msg import HomePosition  # Correct message type for /mavros/home_position/home
from geographic_msgs.msg import GeoPointStamped  # Correct message type for /mavros/global_position/gp_origin
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Define a QoS profile that matches the publisher's settings
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Match the publisher's reliability policy
    history=QoSHistoryPolicy.KEEP_LAST,            # Keep the last N messages
    depth=10                                       # Queue size
)

class TopicChecker(Node):
    def __init__(self):
        super().__init__('topic_checker')
        self.gp_origin_received = False
        self.home_position_received = False

        # Subscribe to /mavros/global_position/gp_origin (GeoPointStamped)
        self.gp_origin_sub = self.create_subscription(
            msg_type=GeoPointStamped,
            topic='/mavros/global_position/gp_origin',
            callback=self.gp_origin_callback,
            qos_profile=qos_profile
        )

        # Subscribe to /mavros/home_position/home (HomePosition)
        self.home_position_sub = self.create_subscription(
            msg_type=HomePosition,
            topic='/mavros/home_position/home',
            callback=self.home_position_callback,
            qos_profile=qos_profile
        )

    def gp_origin_callback(self, msg):
        self.gp_origin_received = True
        self.get_logger().info("Received message on /mavros/global_position/gp_origin")

    def home_position_callback(self, msg):
        self.home_position_received = True
        self.get_logger().info("Received message on /mavros/home_position/home")

    def reset_flags(self):
        self.gp_origin_received = False
        self.home_position_received = False

def launch_mocap_to_vision_pose():
    # Use subprocess to launch the file and keep it running
    process = subprocess.Popen(
        ['ros2', 'launch', 'mocap_to_vision_pose_ros2', 'mocap_to_vision_pose.launch.py'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    return process

def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    topic_checker = TopicChecker()
    executor.add_node(topic_checker)

    max_attempts = 10
    attempt = 0
    launch_process = None

    while attempt < max_attempts:
        attempt += 1
        print(f"Attempt {attempt} of {max_attempts}")

        # Reset flags
        topic_checker.reset_flags()

        # Launch the launch file
        print("Launching mocap_to_vision_pose.launch.py...")
        launch_process = launch_mocap_to_vision_pose()

        # Wait for 5 seconds to check for messages
        start_time = time.time()
        while time.time() - start_time < 5:
            executor.spin_once(timeout_sec=0.1)
            if topic_checker.gp_origin_received and topic_checker.home_position_received:
                print("Both topics received messages. Keeping the launch file running.")
                # Stop checking but keep the launch file running
                while rclpy.ok():
                    pass  # Keep the node alive
                return

        print("No messages received on both topics within 5 seconds. Relaunching...")
        if launch_process:
            launch_process.terminate()  # Terminate the previous launch process

    print("Max attempts reached. Exiting.")
    if launch_process:
        launch_process.terminate()  # Terminate the launch process
    rclpy.shutdown()

if __name__ == '__main__':
    main()
