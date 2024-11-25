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
                            thread_id: int = 0,
                            show_info: bool = True):
        if len(joint_pos_left_deg) != len(joint_pos_right_deg):
            self.get_logger().error("The number of joints in left is different from that in right.")
            self.destroy_node()
            return

        if self._stop_event.is_set():
            if show_info:
                self.get_logger().warning(
                    f"Publishing thread {thread_id} stopped.")
            return

        if show_info:
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

            if show_info:
                self.get_logger().info(
                    f"Published LEFT joint data.\n{joint_pos_left_deg}")
            self._joint_trajectory_publisher_left.publish(msg_left)
            if show_info:
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

    def pub_arm(self,
                joint_pos_left_deg: list,
                joint_pos_right_deg: list,
                thread_id: int = 0,
                show_info: bool = True):
        # Stop the current publishing thread if it's running
        with self._publish_lock:
            if self._publish_thread and self._publish_thread.is_alive():
                self._stop_event.set()
                self._publish_thread.join()

            # Reset the stop event and start a new publishing thread
            self._stop_event.clear()
            self._publish_thread = threading.Thread(
                target=self._publish_trajectory,
                args=(joint_pos_left_deg, joint_pos_right_deg,
                      thread_id, show_info)
            )
            self._publish_thread.start()

    def _cal_pub_frame_list_by_speed(self,
                                     dest_left_joint_deg_angle: list,
                                     dest_right_joint_deg_angle: list,
                                     speed: float,
                                     fps: float) -> tuple[list[list], list[list]]:
        """Calculates the intermediate joint angles for a trajectory based on the max speed.

        The duration is calculated based on the farthest distance between the joints,
        and the speed is then used to calculate the duration for each joint.
        As a result, the speed is slower for shorter movement distances.

        Args:
            dest_left_joint_deg_angle (list): Target angles for the left joints in degrees.
            dest_right_joint_deg_angle (list): Target angles for the right joints in degrees.
            speed (float): Movement speed in degrees per second.
            fps (float): Frames per second for the publisher.

        Returns:
            tuple[list[list], list[list]]: Two lists of intermediate joint angles for the
                left and right joints, respectively, for each frame in the publisher.
        """
        # Calculate the joint position differences
        diff_left: np.ndarray = np.subtract(dest_left_joint_deg_angle,
                                            self.prev_left_joint_deg_angle)
        diff_right: np.ndarray = np.subtract(
            dest_right_joint_deg_angle, self.prev_right_joint_deg_angle)

        # Calculate maximum possible frames based on speed constraint
        duration_left: np.ndarray = np.absolute(diff_left) / speed
        duration_right: np.ndarray = np.absolute(diff_right) / speed
        max_frames_left: float = max(duration_left * fps)
        max_frames_right: float = max(duration_right * fps)
        # Total frames to publish
        num_frames: int = int(max(max_frames_left, max_frames_right))

        # Generate intermediate joint angles for each frame
        left_frames = [self.prev_left_joint_deg_angle +
                       (diff_left * (i / num_frames)) for i in range(1, num_frames + 1)]
        right_frames = [self.prev_right_joint_deg_angle +
                        (diff_right * (i / num_frames)) for i in range(1, num_frames + 1)]

        return left_frames, right_frames

    def _cal_pub_frame_list_by_duration(self,
                                        dest_left_joint_deg_angle: list,
                                        dest_right_joint_deg_angle: list,
                                        duration: float,
                                        fps: float) -> tuple[list[list], list[list]]:
        """Calculates the intermediate joint angles for a trajectory based on duration.

        The duration is the same for all joint movements, so the speed increases when the movement distance is longer.

        Args:
            dest_left_joint_deg_angle (list): Target angles for the left joints in degrees.
            dest_right_joint_deg_angle (list): Target angles for the right joints in degrees.
            duration (float): Total duration of the publisher in seconds.
            fps (float): Frames per second for the publisher.

        Returns:
            tuple[list[list], list[list]]: Two lists of intermediate joint angles for the 
            left and right joints, respectively, for each frame in the publisher.
        """
        # Calculate the joint position differences
        diff_left: np.ndarray = np.subtract(dest_left_joint_deg_angle,
                                            self.prev_left_joint_deg_angle)
        diff_right: np.ndarray = np.subtract(
            dest_right_joint_deg_angle, self.prev_right_joint_deg_angle)

        # Total frames to publish
        num_frames: int = int(duration * fps)

        # Generate intermediate joint angles for each frame
        left_frames = [self.prev_left_joint_deg_angle +
                       (diff_left * (i / num_frames)) for i in range(1, num_frames + 1)]
        right_frames = [self.prev_right_joint_deg_angle +
                        (diff_right * (i / num_frames)) for i in range(1, num_frames + 1)]

        return left_frames, right_frames

    def _pub_trajectory_with_param(self,
                                   dest_left_joint_deg_angle: list,
                                   dest_right_joint_deg_angle: list,
                                   param: float = DEFAULT_SPEED_DEG_PER_SEC,
                                   is_speed: bool = True,
                                   fps: float = DEFAULT_FPS,
                                   thread_id: int = 0,
                                   show_info: bool = True):
        """Publishes a trajectory to move the joints using the specified parameter.

        Args:
            dest_left_joint_deg_angle (list): Target angles for the left joints in degrees.
            dest_right_joint_deg_angle (list): Target angles for the right joints in degrees.
            param (float, optional): Movement speed in degrees per second if `is_speed` is True, 
                otherwise the duration of the publisher in seconds. Defaults to DEFAULT_SPEED_DEG_PER_SEC.
            is_speed (bool, optional): Indicates whether `param` represents speed or duration. Defaults to True,
                which means the meaning of the param is movement speed in degrees.
            fps (float, optional): Frames per second for the publisher. Defaults to DEFAULT_FPS.
            thread_id (int, optional): Identifier for the thread publishing the publisher. Defaults to 0.
            show_info (bool, optional): Whether to log information about the thread stopping. Defaults to True.
        """
        # Calculate frames for each step in the movement
        if is_speed:
            left_frames, right_frames = self._cal_pub_frame_list_by_speed(
                dest_left_joint_deg_angle,
                dest_right_joint_deg_angle,
                param,
                fps
            )
        else:
            left_frames, right_frames = self._cal_pub_frame_list_by_duration(
                dest_left_joint_deg_angle,
                dest_right_joint_deg_angle,
                param,
                fps
            )

        # Publish each frame, checking the stop_event before each publish
        for left_frame, right_frame in zip(left_frames, right_frames):
            if self._stop_event.is_set():
                if show_info:
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

    def pub_arm_with_param(self,
                           dest_left_joint_deg_angle: list,
                           dest_right_joint_deg_angle: list,
                           param: float = DEFAULT_SPEED_DEG_PER_SEC,
                           is_speed: bool = True,
                           fps: float = DEFAULT_FPS,
                           thread_id: int = 0,
                           show_info: bool = True):
        """Starts a new thread to publish a trajectory for the joints using the specified parameter.

        Args:
            dest_left_joint_deg_angle (list): Target angles for the left joints in degrees.
            dest_right_joint_deg_angle (list): Target angles for the right joints in degrees.
            param (float, optional): Movement speed in degrees per second if `is_speed` is True, 
                otherwise the duration of the publisher in seconds. Defaults to DEFAULT_SPEED_DEG_PER_SEC.
            is_speed (bool, optional): Indicates whether `param` represents speed or duration. Defaults to True,
                which means the meaning of the param is movement speed in degrees.
            fps (float, optional): Frames per second for the publisher. Defaults to DEFAULT_FPS.
            thread_id (int, optional): Identifier for the thread publishing the publisher. Defaults to 0.
            show_info (bool, optional): Whether to log information about the thread stopping. Defaults to True.
        """
        # Stop the current publishing thread if it's running
        with self._publish_lock:
            if self._publish_thread and self._publish_thread.is_alive():
                self._stop_event.set()
                self._publish_thread.join()

            # Reset the stop event and start a new publishing thread
            self._stop_event.clear()
            self._publish_thread = threading.Thread(
                target=self._pub_trajectory_with_param,
                args=(
                    dest_left_joint_deg_angle,
                    dest_right_joint_deg_angle,
                    param,
                    is_speed,
                    fps,
                    thread_id,
                    show_info
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
        arm_publisher.pub_arm_with_param(MIN_LEFT_JOINT_DEG_ANGLE,
                                         MIN_RIGHT_JOINT_DEG_ANGLE,
                                         param=DEFAULT_DURATION_SEC,
                                         is_speed=False,
                                         fps=DEFAULT_FPS,
                                         thread_id=0,
                                         show_info=True)
        time.sleep(0.5)
        arm_publisher.get_logger().info("-------------------------------------")
        arm_publisher.get_logger().info("----- Published MAX joint data. -----")
        arm_publisher.get_logger().info("-------------------------------------")
        arm_publisher.pub_arm_with_param(MAX_LEFT_JOINT_DEG_ANGLE,
                                         MAX_RIGHT_JOINT_DEG_ANGLE,
                                         param=DEFAULT_DURATION_SEC,
                                         is_speed=False,
                                         fps=DEFAULT_FPS,
                                         thread_id=1,
                                         show_info=True)
        time.sleep(0.5)
        arm_publisher.get_logger().info("-------------------------------------")
        arm_publisher.get_logger().info("--- Published DEFAULT joint data. ---")
        arm_publisher.get_logger().info("-------------------------------------")
        arm_publisher.pub_arm_with_param(DEFAULT_LEFT_JOINT_DEG_ANGLE,
                                         DEFAULT_RIGHT_JOINT_DEG_ANGLE,
                                         param=DEFAULT_SPEED_DEG_PER_SEC,
                                         is_speed=True,
                                         fps=DEFAULT_FPS,
                                         thread_id=2,
                                         show_info=True)
        time.sleep(18.5)
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
