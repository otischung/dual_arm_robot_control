import math
import numpy as np
import threading
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from arm_control.params import *


class ArmPublisher(Node):
    def __init__(self):
        super().__init__('arm_publisher')
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

    def pub_arm(self, joint_pos_left_deg: list, joint_pos_right_deg: list):
        if len(joint_pos_left_deg) != len(joint_pos_right_deg):
            self.get_logger().error("The number of joints in left is different from that in right.")
            return None

        msg_left = JointTrajectoryPoint()
        msg_left.positions = [math.radians(pos) for pos in joint_pos_left_deg]
        # Replace with actual desired velocities
        msg_left.velocities = [0.0 for _ in joint_pos_left_deg]
        msg_right = JointTrajectoryPoint()
        msg_right.positions = [math.radians(pos)
                               for pos in joint_pos_right_deg]
        # Replace with actual desired velocities
        msg_right.velocities = [0.0 for _ in joint_pos_right_deg]
        self._joint_trajectory_publisher_left.publish(msg_left)
        self._joint_trajectory_publisher_right.publish(msg_right)


class ArmControllerThread(Node):
    """A controller class to manage threaded publishing of arm joint positions
    using the ArmPublisher, ensuring that previous threads are stopped
    before new ones start.

    Attributes:
        prev_left_joint_deg_angle (list): Previous joint angles for the left arm in degrees.
        prev_right_joint_deg_angle (list): Previous joint angles for the right arm in degrees.
        arm_publisher (ArmPublisher): Instance of the ArmPublisher to handle publishing.
        active_thread (threading.Thread): The currently active thread for the arm publishing.
        stop_event (threading.Event): Event to signal thread stopping for new publish requests.
    """

    def __init__(
            self,
            init_left_joint_deg_angle: list,
            init_right_joint_deg_angle: list):
        """Initializes the ArmControllerThread with initial joint angles,
        arm publisher, and threading controls.

        Args:
            init_left_joint_deg_angle (list): Initial joint angles for the left arm in degrees.
            init_right_joint_deg_angle (list): Initial joint angles for the right arm in degrees.
        """
        super().__init__('arm_controller_thread')

        # Record the previous joints' angles
        self.prev_left_joint_deg_angle = init_left_joint_deg_angle
        self.prev_right_joint_deg_angle = init_right_joint_deg_angle

        # Initialize the ArmPublisher and threading controls
        self.arm_publisher = ArmPublisher()
        self.active_thread = None
        self.stop_event = threading.Event()

    def _cal_pub_frame_list(self,
                            dest_left_joint_deg_angle: list,
                            dest_right_joint_deg_angle: list,
                            speed: float,
                            duration: float,
                            fps: float) -> tuple[list[list], list[list]]:
        """Calculates the joint angles for left and right arms for each publish frame.

        Args:
            dest_left_joint_deg_angle (list): The destination angles for the left joints in degrees.
            dest_right_joint_deg_angle (list): The destination angles for the right joints in degrees.
            speed (float): The speed (deg/s) at which each joint should move.
            duration (float): The total duration (s) for the motion.
            fps (float): The frames per second (Hz) for publishing.

        Returns:
            tuple[list[list], list[list]]: A tuple containing two lists of lists:
                - Left arm joint positions for each frame.
                - Right arm joint positions for each frame.
        """
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

    def _pub_arm_thread(self,
                        dest_left_joint_deg_angle: list,
                        dest_right_joint_deg_angle: list,
                        speed: float = 5.0,
                        duration: float = 1.0,
                        fps: float = 10.0):
        """Publishes arm joint positions in a separate thread, periodically
        checking the stop event to determine if the thread should stop.

        Args:
            dest_left_joint_deg_angle (list): Target joint angles for the left arm in degrees.
            dest_right_joint_deg_angle (list): Target joint angles for the right arm in degrees.
            speed (float): The speed (deg/s) for the arm movement.
            duration (float): The total duration (s) for the motion.
            fps (float): The frames per second (Hz) for publishing.
        """
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
            if self.stop_event.is_set():
                print("Thread stopped.")
                return  # Exit if stop event is set

            # Publish the current frame
            self.arm_publisher.pub_arm(left_frame, right_frame)
            print(f"Published left: {left_frame}, right: {right_frame}")

            # Delay for the time between frames
            time.sleep(1.0 / fps)

        # Update previous angles to the final destination angles
        self.prev_left_joint_deg_angle = dest_left_joint_deg_angle
        self.prev_right_joint_deg_angle = dest_right_joint_deg_angle

    def pub_arm_thread(self,
                       dest_left_joint_deg_angle: list,
                       dest_right_joint_deg_angle: list,
                       speed: float = 5.0,
                       duration: float = 1.0,
                       fps: float = 10.0):
        """Starts a new thread to publish arm joint positions. Stops any existing
        publishing thread before starting a new one.

        Args:
            dest_left_joint_deg_angle (list): Target joint angles for the left arm in degrees.
            dest_right_joint_deg_angle (list): Target joint angles for the right arm in degrees.
            speed (float, optional): The speed (deg/s) for the arm movement. Defaults to 5.0.
            duration (float, optional): The total duration (s) for the motion. Defaults to 1.0.
            fps (float, optional): The frames per second (Hz) for publishing. Defaults to 10.0.
        """
        # If an existing thread is running, signal it to stop
        if self.active_thread and self.active_thread.is_alive():
            print("Stopping the current thread.")
            self.stop_event.set()
            self.active_thread.join()  # Wait for the thread to finish

        # Reset the stop event for the new thread
        self.stop_event.clear()

        # Create a new thread for publishing arm positions
        self.active_thread = threading.Thread(
            target=self._pub_arm_thread,
            args=(dest_left_joint_deg_angle,
                  dest_right_joint_deg_angle,
                  speed,
                  duration,
                  fps)
        )

        # Start the new thread
        self.active_thread.start()
        print("Started a new publishing thread.")


def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the ArmController class
    controller = ArmControllerThread(
        DEFAULT_LEFT_JOINT_DEG_ANGLE, DEFAULT_RIGHT_JOINT_DEG_ANGLE)
    
    # Start rclpy spinning to keep the node alive
    rclpy.spin(controller)

    # Start publishing to the arm with specific joint positions
    controller.pub_arm_thread(MIN_LEFT_JOINT_DEG_ANGLE,
                              MIN_RIGHT_JOINT_DEG_ANGLE)
    time.sleep(2)  # Wait for 2 seconds

    # Call again to stop the previous call and start a new one
    controller.pub_arm_thread(MAX_LEFT_JOINT_DEG_ANGLE,
                              MAX_RIGHT_JOINT_DEG_ANGLE)

    # Shutdown sequence after spin is interrupted
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
