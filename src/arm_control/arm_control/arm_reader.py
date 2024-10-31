import copy
import math
import orjson
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import JointState
from serial import Serial
from arm_control.params import *
from arm_control.utils import *


class ArmSerialReader(Node):
    """A ROS 2 node that reads serial data from two ESP32 devices controlling robotic arms and publishes it as joint states.

    This node establishes serial communication with the left and right ESP32 devices, receives the joint angle data
    in degrees, converts them to radians, and publishes the converted data to topics as `JointState` messages.

    Attributes:
        _serial_left (Serial): The serial connection to the left ESP32 device.
        _serial_right (Serial): The serial connection to the right ESP32 device.
        _left_publisher (Publisher): Publisher for the joint state data of the left arm.
        _right_publisher (Publisher): Publisher for the joint state data of the right arm.
        _timer (Timer): Timer to periodically trigger the reader_callback.
        timer_period (float): Timer period for the callback execution.
        log_interval (Duration): The duration for logging interval.
        last_log_time (Time): Last time when log messages were printed.

    Methods:
        __init__():
            Initializes the node, declares parameters for serial ports, sets up serial communication, and
            creates publishers and a timer for reading serial data.

        reader_callback():
            Reads serial data from the ESP32 devices, decodes it, converts the values to radians, and publishes
            the joint states to the appropriate topics. Logs the data periodically.

    Raises:
        orjson.JSONDecodeError: If there is an error decoding the JSON data from the ESP32.
        KeyError: If a required key is missing in the decoded JSON data.
        UnicodeDecodeError: If the received serial data contains invalid characters.

    Example:
        To run this node, initialize it and spin:

        >>> rclpy.init()
        >>> serial_reader = ArmSerialReader()
        >>> rclpy.spin(serial_reader)
    """

    def __init__(self):
        """Initializes the ArmSerialReader node and sets up parameters, serial connections, publishers, and timers.

        This method declares parameters for the left and right ESP32 serial ports, establishes serial connections to
        both devices, and creates publishers to publish joint states of both arms. It also initializes a timer to
        periodically call the `reader_callback` method.
        """
        super().__init__('arm_serial_reader')

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
        self.declare_parameter(
            "left", DEFAULT_LEFT_USB_SERIAL, left_descriptor)
        self.declare_parameter(
            "right", DEFAULT_RIGHT_USB_SERIAL, right_descriptor)

        # Retrieve serial port values for the ESP32 devices.
        serial_port_left = self.get_parameter("left").value
        serial_port_right = self.get_parameter("right").value

        # Log the serial port settings.
        self.get_logger().info("--------------------------------------")
        self.get_logger().info(f"Setting left to {serial_port_left}")
        self.get_logger().info(f"Setting right to {serial_port_right}")
        self.get_logger().info("--------------------------------------")

        # Set up the serial connections to the left and right ESP32 devices.
        self._serial_left = Serial(
            serial_port_left, BAUD_RATE, timeout=0)
        self._serial_right = Serial(
            serial_port_right, BAUD_RATE, timeout=0)

        # Create publishers for joint states from the left and right ESP32 devices.
        self._left_publisher = self.create_publisher(
            JointState, LEFT_JOINTS_STATE_TOPIC, ROS_QOS_DEPTH)
        self._right_publisher = self.create_publisher(
            JointState, RIGHT_JOINTS_STATE_TOPIC, ROS_QOS_DEPTH)

        # Set timer period and callback.
        self.timer_period = READER_TIMER_PERIOD  # seconds
        self._timer = self.create_timer(READER_CALLBACK_TIMER_PERIOD, self.reader_callback)

        # Set log interval and initialize logging time.
        self.log_interval = Duration(seconds=self.timer_period)
        current_time = self.get_clock().now()
        self.last_log_time = current_time

    def reader_callback(self):
        """Callback function to read and process serial data from ESP32 devices.

        This method reads data from the left and right ESP32 devices, decodes the received JSON data,
        converts the joint angles from degrees to radians, and publishes them as `JointState` messages
        on the respective topics. Logs the received data periodically.

        Raises:
            orjson.JSONDecodeError: If there is an error decoding the JSON data from the serial input.
            KeyError: If the required key is missing in the JSON data.
            UnicodeDecodeError: If the serial data contains invalid Unicode characters.
        """
        # Read data from the serial devices.
        data_left = self._serial_left.readline()
        data_right = self._serial_right.readline()

        # Process and log the data from the left ESP32 device.
        try:
            degree_data_left = orjson.loads(data_left)
            degree_positions_left = degree_data_left[SERIAL_JSON_KEY]
        except orjson.JSONDecodeError as error:
            self.get_logger().error(
                f"{bcolors.FAIL}LEFT: Json decode error when recv {data_left}: {error}{bcolors.ENDC}")
            return
        except KeyError as error:
            self.get_logger().error(
                f"{bcolors.FAIL}LEFT: KeyError when recv {degree_data_left}: {error}{bcolors.ENDC}")
            return
        except UnicodeDecodeError as error:
            self.get_logger().error(
                f"{bcolors.FAIL}LEFT: UnicodeDecodeError when recv {data_left}: {error}{bcolors.ENDC}")
            return

        # Process and log the data from the right ESP32 device.
        try:
            degree_data_right = orjson.loads(data_right)
            degree_positions_right = degree_data_right[SERIAL_JSON_KEY]
        except orjson.JSONDecodeError as error:
            self.get_logger().error(
                f"{bcolors.FAIL}RIGHT: Json decode error when recv {data_right}: {error}{bcolors.ENDC}")
            return
        except KeyError as error:
            self.get_logger().error(
                f"{bcolors.FAIL}RIGHT: KeyError when recv {degree_data_right}: {error}{bcolors.ENDC}")
            return
        except UnicodeDecodeError as error:
            self.get_logger().error(
                f"{bcolors.FAIL}RIGHT: UnicodeDecodeError when recv {data_right}: {error}{bcolors.ENDC}")
            return

        # Convert degree positions to radians.
        radian_positions_left = [math.radians(
            deg) for deg in degree_positions_left]
        radian_positions_right = [math.radians(
            deg) for deg in degree_positions_right]

        # Create joint names using the number of joint angles.
        _joint_names = [f"J{i + 1}" for i in range(len(radian_positions_left))]

        # Create and publish joint state messages.
        msg_left = JointState()
        msg_left.header.stamp = self.get_clock().now().to_msg()
        msg_left.name = _joint_names
        msg_left.position = radian_positions_left
        msg_left.velocity = []
        msg_left.effort = []

        msg_right = copy.deepcopy(msg_left)
        msg_right.position = radian_positions_right

        self._left_publisher.publish(msg_left)
        self._right_publisher.publish(msg_right)

        # Log the received data at the specified interval.
        current_time = self.get_clock().now()
        if current_time - self.last_log_time >= self.log_interval:
            self.get_logger().info(
                f'{bcolors.OKGREEN}LEFT: Receive from arm esp32: {degree_data_left}{bcolors.ENDC}')
            self.get_logger().info(
                f'{bcolors.OKBLUE}RIGHT Receive from arm esp32: {degree_data_right}{bcolors.ENDC}')
            self.last_log_time = current_time


def main(args=None):
    """Main entry point for the ArmSerialReader node.

    This function initializes the ROS 2 Python client library, creates the ArmSerialReader node, and
    spins it to keep it alive, processing incoming serial data from the ESP32 devices.
    """
    rclpy.init(args=args)

    serial_reader = ArmSerialReader()
    rclpy.spin(serial_reader)

    # Cleanup after the node is destroyed.
    serial_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
