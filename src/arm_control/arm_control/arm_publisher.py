import math
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
        msg_left.velocities = [0.0 for _ in joint_pos_left_deg]  # Replace with actual desired velocities
        msg_right = JointTrajectoryPoint()
        msg_right.positions = [math.radians(pos) for pos in joint_pos_right_deg]
        msg_right.velocities = [0.0 for _ in joint_pos_right_deg]  # Replace with actual desired velocities
        self._joint_trajectory_publisher_left.publish(msg_left)
        self._joint_trajectory_publisher_right.publish(msg_right)
