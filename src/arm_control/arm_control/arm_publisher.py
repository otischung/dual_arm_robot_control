import math
import numpy as np
import threading
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.executors import MultiThreadedExecutor
from arm_control.params import *


class ArmPublisher(Node):
    def __init__(self, node_name: str = "arm_publisher"):
        super().__init__(node_name)
        self._joint_trajectory_publisher_left = self.create_publisher(
            JointTrajectoryPoint,
            LEFT_JOINTS_TOPIC,
            ROS_QOS_DEPTH
        )
        self._joint_trajectory_publisher_right = self.create_publisher(
            JointTrajectoryPoint,
            RIGHT_JOINTS_TOPIC,
            ROS_QOS_DEPTH
        )
        self._publish_thread = None
        self._publish_lock = threading.Lock()
        self._stop_event = threading.Event()

    def _publish_trajectory(self, joint_pos_left_deg: list, joint_pos_right_deg: list):
        try:
            msg_left = JointTrajectoryPoint()
            msg_left.positions = [math.radians(
                pos) for pos in joint_pos_left_deg]
            msg_left.velocities = [0.0 for _ in joint_pos_left_deg]

            msg_right = JointTrajectoryPoint()
            msg_right.positions = [math.radians(
                pos) for pos in joint_pos_right_deg]
            msg_right.velocities = [0.0 for _ in joint_pos_right_deg]

            # Simulating some time-consuming process
            for _ in range(5):  # Example: 5 iterations of publishing
                if self._stop_event.is_set():
                    self.get_logger().info("Publishing thread stopped.")
                    return
                self._joint_trajectory_publisher_left.publish(msg_left)
                self._joint_trajectory_publisher_right.publish(msg_right)
                self.get_logger().info("Published joint data.")
                time.sleep(1)  # Simulate delay between publishing
        except Exception as e:
            self.get_logger().error(f"Error in publish thread: {e}")

    def destroy_node(self):
        # Clean up the thread safely during node destruction
        with self._publish_lock:
            if self._publish_thread and self._publish_thread.is_alive():
                self._stop_event.set()
                self._publish_thread.join()
        super().destroy_node()

    def pub_arm(self, joint_pos_left_deg: list, joint_pos_right_deg: list):
        if len(joint_pos_left_deg) != len(joint_pos_right_deg):
            self.get_logger().error("The number of joints in left is different from that in right.")
            return

        # Stop the current publishing thread if it's running
        with self._publish_lock:
            if self._publish_thread and self._publish_thread.is_alive():
                self._stop_event.set()
                self._publish_thread.join()

            # Reset the stop event and start a new publishing thread
            self._stop_event.clear()
            self._publish_thread = threading.Thread(
                target=self._publish_trajectory,
                args=(joint_pos_left_deg, joint_pos_right_deg)
            )
            self._publish_thread.start()


def main(args=None):
    rclpy.init(args=args)

    try:
        # Create the node
        arm_publisher = ArmPublisher()

        # Use a MultiThreadedExecutor to support threading in nodes
        executor = MultiThreadedExecutor()
        executor.add_node(arm_publisher)

        # Start the executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # Simulate sending joint positions at different intervals
        time.sleep(1)
        # Example target joint angles
        arm_publisher.pub_arm(MIN_LEFT_JOINT_DEG_ANGLE,
                              MIN_RIGHT_JOINT_DEG_ANGLE)
        time.sleep(6)
        # Update with new joint angles
        arm_publisher.pub_arm(MAX_LEFT_JOINT_DEG_ANGLE,
                              MAX_RIGHT_JOINT_DEG_ANGLE)
        time.sleep(3)
        # Reset to default positions
        arm_publisher.pub_arm(DEFAULT_LEFT_JOINT_DEG_ANGLE,
                              DEFAULT_RIGHT_JOINT_DEG_ANGLE)

        # Allow time for publishing before shutting down
        time.sleep(5)
    except KeyboardInterrupt:
        pass
    finally:
        # Shut down the node and executor properly
        arm_publisher.destroy_node()
        executor.shutdown()
        executor_thread.join()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
