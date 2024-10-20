import functools
import math
import orjson
import params
import rclpy
from utils import bcolors
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from serial import Serial
from trajectory_msgs.msg import JointTrajectoryPoint


class ArmSerialWriter(Node):
    """A ROS 2 node that handles serial communication with two ESP32 devices controlling robotic arms.

    This node subscribes to joint angle topics, converts the angles from radians to degrees,
    and sends the corresponding target angles to the ESP32 devices controlling the left and right arms
    via serial communication.

    Attributes:
        _serial_left (Serial): The serial connection to the left ESP32 device.
        _serial_right (Serial): The serial connection to the right ESP32 device.
        _subscriber_left (Subscription): Subscription to the left arm's joint angles topic.
        _subscriber_right (Subscription): Subscription to the right arm's joint angles topic.

    Methods:
        __init__():
            Initializes the node, declares parameters for serial port devices, sets up serial communication,
            and subscribes to the left and right arm joint angle topics.

        listener_callback(arm_side: params.ArmSide, msg: JointTrajectoryPoint):
            Callback function that receives joint angles, converts them to degrees, encodes them as JSON, and
            sends them to the appropriate ESP32 device over serial.

    Raises:
        orjson.JSONEncodeError: If there is an error encoding the joint angles to JSON format.

    Example:
        To run this node, initialize it and spin:

        >>> rclpy.init()
        >>> serial_writer = ArmSerialWriter()
        >>> rclpy.spin(serial_writer)
    """
    def __init__(self):
        """Initializes the ArmSerialWriter node and sets up parameters and subscriptions.

        This method declares parameters for the serial port devices for both the left and right ESP32 controllers.
        It then establishes serial connections to both controllers and subscribes to the joint angle topics
        for both the left and right robotic arms.
        """
        super().__init__('arm_serial_writer')

        # Declare parameters for left and right ESP32 device serial ports.
        left_descriptor = ParameterDescriptor(
            name="left",
            type=ParameterType.PARAMETER_STRING,
            description="This defines the device of the left ESP32.",
        )
        right_descriptor = ParameterDescriptor(
            name="right",
            type=ParameterType.PARAMETER_STRING,
            description="This defines the device of the right ESP32.",
        )
        self.declare_parameter("left", params.DEFAULT_LEFT_USB_SERIAL, left_descriptor)
        self.declare_parameter("right", params.DEFAULT_RIGHT_USB_SERIAL, right_descriptor)

        # Retrieve serial port values for the ESP32 devices.
        serial_port_left = self.get_parameter("left").value
        serial_port_right = self.get_parameter("right").value

        # Log the serial port settings.
        self.get_logger().info("--------------------------------------")
        self.get_logger().info(f"Setting left to {serial_port_left}")
        self.get_logger().info(f"Setting right to {serial_port_right}")
        self.get_logger().info("--------------------------------------")

        # Set up the serial connections to the left and right ESP32 devices.
        self._serial_left = Serial(serial_port_left, params.BAUD_RATE, timeout=0)
        self._serial_right = Serial(serial_port_right, params.BAUD_RATE, timeout=0)

        # Subscribe to the joint angle topics for both left and right arms.
        self._subscriber_left = self.create_subscription(
            JointTrajectoryPoint,
            params.LEFT_JOINTS_TOPIC,
            functools.partial(self.listener_callback, params.ArmSide.LEFT),
            params.ROS_QOS_DEPTH
        )
        self._subscriber_right = self.create_subscription(
            JointTrajectoryPoint,
            params.RIGHT_JOINTS_TOPIC,
            functools.partial(self.listener_callback, params.ArmSide.RIGHT),
            params.ROS_QOS_DEPTH
        )

    def listener_callback(self, arm_side: params.ArmSide, msg: JointTrajectoryPoint):
        """Callback function to process incoming joint angle messages.

        This method receives joint angles in radians, converts them to degrees, and sends the angles
        as a JSON string to the corresponding ESP32 device over serial.

        Args:
            arm_side (params.ArmSide): The side of the arm (LEFT or RIGHT) that the message corresponds to.
            msg (JointTrajectoryPoint): The message containing joint angle positions in radians.

        Raises:
            orjson.JSONEncodeError: If there is an error encoding the joint angles to JSON.
        """
        radian_positions = msg.positions
        degree_positions = [math.degrees(rad) % 360 for rad in radian_positions]
        ctrl_json = {"servo_target_angles": degree_positions}

        # Handle the left arm's joint angles.
        if arm_side == params.ArmSide.LEFT:
            self.get_logger().info(f"LEFT: Receive from {params.LEFT_JOINTS_TOPIC}: {radian_positions}")
            try:
                ctrl_str = orjson.dumps(ctrl_json, option=orjson.OPT_APPEND_NEWLINE)
                self._serial_left.write(ctrl_str)
            except orjson.JSONEncodeError as error:
                self.get_logger().error(f"{bcolors.FAIL}LEFT: Json encode error when recv message: {msg}: {error}{bcolors.ENDC}")
                return
            self.get_logger().info(f"LEFT: Send to {params.DEFAULT_LEFT_USB_SERIAL}: {ctrl_str}")
        
        # Handle the right arm's joint angles.
        elif arm_side == params.ArmSide.RIGHT:
            self.get_logger().info(f"RIGHT: Receive from {params.RIGHT_JOINTS_TOPIC}: {radian_positions}")
            try:
                ctrl_str = orjson.dumps(ctrl_json, option=orjson.OPT_APPEND_NEWLINE)
                self._serial_right.write(ctrl_str)
            except orjson.JSONEncodeError as error:
                self.get_logger().error(f"{bcolors.FAIL}RIGHT: Json encode error when recv message: {msg}: {error}{bcolors.ENDC}")
                return
            self.get_logger().info(f"RIGHT: Send to {params.DEFAULT_RIGHT_USB_SERIAL}: {ctrl_str}")
            


def main(args=None):
    """Main entry point for the ArmSerialWriter node.

    This function initializes the ROS 2 Python client library, creates the ArmSerialWriter node, 
    and spins it to keep it alive, processing incoming joint angle messages.
    """ 
    rclpy.init(args=args)
    serial_writer = ArmSerialWriter()
    rclpy.spin(serial_writer)

    # Cleanup after the node is destroyed.
    serial_writer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()