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
    def __init__(self,
                 node_name: str = "arm_publisher",
                 init_left_joint_deg_angle: list = DEFAULT_LEFT_JOINT_DEG_ANGLE,
                 init_right_joint_deg_angle: list = DEFAULT_RIGHT_JOINT_DEG_ANGLE):
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
        self.prev_left_joint_deg_angle = init_left_joint_deg_angle
        self.prev_right_joint_deg_angle = init_right_joint_deg_angle
        self._publish_thread = None
        self._publish_lock = threading.Lock()
        self._stop_event = threading.Event()

    def _publish_trajectory(self,
                            joint_pos_left_deg: list,
                            joint_pos_right_deg: list,
                            thread_id: int = 0):
        if len(joint_pos_left_deg) != len(joint_pos_right_deg):
            self.get_logger().error("The number of joints in left is different from that in right.")
            self.destroy_node()
            return

        if self._stop_event.is_set():
            self.get_logger().warning(
                f"Publishing thread {thread_id} stopped.")
            return

        self.get_logger().info(f"THREAD ID: {thread_id}")
        try:
            msg_left = JointTrajectoryPoint()
            msg_left.positions = [math.radians(
                pos) for pos in joint_pos_left_deg]
            msg_left.velocities = [0.0 for _ in joint_pos_left_deg]

            msg_right = JointTrajectoryPoint()
            msg_right.positions = [math.radians(
                pos) for pos in joint_pos_right_deg]
            msg_right.velocities = [0.0 for _ in joint_pos_right_deg]

            self.get_logger().info(
                f"Published LEFT joint data.\n{joint_pos_left_deg}")
            self._joint_trajectory_publisher_left.publish(msg_left)
            self.get_logger().info(
                f"Published RIGHT joint data.\n{joint_pos_right_deg}")
            self._joint_trajectory_publisher_right.publish(msg_right)

        except Exception as e:
            self.get_logger().error(
                f"Error in publish thread {thread_id}: {e}")

    def destroy_node(self):
        # Clean up the thread safely during node destruction
        with self._publish_lock:
            if self._publish_thread and self._publish_thread.is_alive():
                self._stop_event.set()
                self._publish_thread.join()
        super().destroy_node()

    def pub_arm(self, joint_pos_left_deg: list, joint_pos_right_deg: list, thread_id: int = 0):
        # Stop the current publishing thread if it's running
        with self._publish_lock:
            if self._publish_thread and self._publish_thread.is_alive():
                self._stop_event.set()
                self._publish_thread.join()

            # Reset the stop event and start a new publishing thread
            self._stop_event.clear()
            self._publish_thread = threading.Thread(
                target=self._publish_trajectory,
                args=(joint_pos_left_deg, joint_pos_right_deg, thread_id)
            )
            self._publish_thread.start()

    def _cal_pub_frame_list(self,
                            dest_left_joint_deg_angle: list,
                            dest_right_joint_deg_angle: list,
                            speed: float,
                            duration: float,
                            fps: float) -> tuple[list[list], list[list]]:
        num_frames = int(fps * duration)  # Total frames to publish
        time_step = 1.0 / fps  # Time per frame

        # Calculate the joint position differences
        diff_left = np.subtract(dest_left_joint_deg_angle,
                                self.prev_left_joint_deg_angle)
        diff_right = np.subtract(
            dest_right_joint_deg_angle, self.prev_right_joint_deg_angle)

        # Calculate maximum possible frames based on speed constraint
        max_frames_left = max(abs(diff_left) / speed / time_step)
        max_frames_right = max(abs(diff_right) / speed / time_step)
        num_frames = min(num_frames, int(
            max(max_frames_left, max_frames_right)))

        # Generate intermediate joint angles for each frame
        left_frames = [self.prev_left_joint_deg_angle +
                       (diff_left * (i / num_frames)) for i in range(1, num_frames + 1)]
        right_frames = [self.prev_right_joint_deg_angle +
                        (diff_right * (i / num_frames)) for i in range(1, num_frames + 1)]

        return left_frames, right_frames

    def _pub_trajectory_with_speed(self,
                                   dest_left_joint_deg_angle: list,
                                   dest_right_joint_deg_angle: list,
                                   speed: float = 5.0,
                                   duration: float = 1.0,
                                   fps: float = 10.0,
                                   thread_id: int = 0):
        # Calculate frames for each step in the movement
        left_frames, right_frames = self._cal_pub_frame_list(
            dest_left_joint_deg_angle,
            dest_right_joint_deg_angle,
            speed,
            duration,
            fps
        )

        # Publish each frame, checking the stop_event before each publish
        for left_frame, right_frame in zip(left_frames, right_frames):
            if self._stop_event.is_set():
                self.get_logger().warning(
                    f"Publishing thread {thread_id} stopped.")
                return

            # Publish the current frame
            self._publish_trajectory(left_frame, right_frame, thread_id)
            # Update previous angles to the current angles
            self.prev_left_joint_deg_angle = left_frame
            self.prev_right_joint_deg_angle = right_frame

            # Delay for the time between frames
            time.sleep(1.0 / fps)

        # # Update previous angles to the final destination angles
        # self.prev_left_joint_deg_angle = dest_left_joint_deg_angle
        # self.prev_right_joint_deg_angle = dest_right_joint_deg_angle

    def pub_arm_with_speed(self,
                           dest_left_joint_deg_angle: list,
                           dest_right_joint_deg_angle: list,
                           speed: float = 5.0,
                           duration: float = 1.0,
                           fps: float = 10.0,
                           thread_id: int = 0):
        # Stop the current publishing thread if it's running
        with self._publish_lock:
            if self._publish_thread and self._publish_thread.is_alive():
                self._stop_event.set()
                self._publish_thread.join()

            # Reset the stop event and start a new publishing thread
            self._stop_event.clear()
            self._publish_thread = threading.Thread(
                target=self._pub_trajectory_with_speed,
                args=(
                    dest_left_joint_deg_angle,
                    dest_right_joint_deg_angle,
                    speed,
                    duration,
                    fps,
                    thread_id
                )
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

        arm_publisher.get_logger().info("-------------------------------------")
        arm_publisher.get_logger().info("----- Published MIN joint data. -----")
        arm_publisher.get_logger().info("-------------------------------------")
        arm_publisher.pub_arm_with_speed(MIN_LEFT_JOINT_DEG_ANGLE,
                                         MIN_RIGHT_JOINT_DEG_ANGLE,
                                         duration=5.0,
                                         fps=1.0,
                                         thread_id=0)
        time.sleep(2)
        arm_publisher.get_logger().info("-------------------------------------")
        arm_publisher.get_logger().info("----- Published MAX joint data. -----")
        arm_publisher.get_logger().info("-------------------------------------")
        arm_publisher.pub_arm_with_speed(MAX_LEFT_JOINT_DEG_ANGLE,
                                         MAX_RIGHT_JOINT_DEG_ANGLE,
                                         duration=5.0,
                                         fps=1.0,
                                         thread_id=1)
        time.sleep(2)
        arm_publisher.get_logger().info("-------------------------------------")
        arm_publisher.get_logger().info("--- Published DEFAULT joint data. ---")
        arm_publisher.get_logger().info("-------------------------------------")
        arm_publisher.pub_arm_with_speed(DEFAULT_LEFT_JOINT_DEG_ANGLE,
                                         DEFAULT_RIGHT_JOINT_DEG_ANGLE,
                                         duration=5.0,
                                         fps=1.0,
                                         thread_id=2)
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
