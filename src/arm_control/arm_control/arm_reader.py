import copy
import math
import orjson
import params
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import JointState
from serial import Serial


class ArmSerialReader(Node):
    def __init__(self):
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

        # Create a publisher for the serial data
        self._left_publisher = self.create_publisher(JointState, params.LEFT_JOINTS_STATE_TOPIC, params.ROS_QOS_DEPTH)
        self._right_publisher = self.create_publisher(JointState, params.RIGHT_JOINTS_STATE_TOPIC, params.ROS_QOS_DEPTH)
        self.timer_period = params.READER_TIMER_PERIOD  # seconds
        self._timer = self.create_timer(params.READER_CALLBACK_TIMER_PERIOD, self.reader_callback)
        self.log_interval = Duration(seconds=self.timer_period)
        current_time = self.get_clock().now()
        self.last_log_time = current_time

        # TODO dynamic to adjust arm
        self._joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]
        self._position = [0.0, 0.0, 0.0, 0.0, 0.0]

    def reader_callback(self):
        # Read data from the serial device
        data_left = self._serial_left.readline()
        data_right = self._serial_right.readline()
        # LEFT
        try:
            degree_data_left = orjson.loads(data_left)
            degree_positions_left = degree_data_left["servo_current_angles"]  # Assuming this is the key in your JSON

        except orjson.JSONDecodeError as error:
            self.get_logger().error(f"LEFT: Json decode error when recv {data_left}: {error}")
            return
        except KeyError as error:
            self.get_logger().error(f"LEFT: KeyError when recv {degree_data_left}: {error}")
            return
        except UnicodeDecodeError as error:
            self.get_logger().error(f"LEFT: UnicodeDecodeError when recv {data_left}: {error}")
            return
        # RIGHT
        try:
            degree_data_right = orjson.loads(data_right)
            degree_positions_right = degree_data_right["servo_current_angles"]  # Assuming this is the key in your JSON

        except orjson.JSONDecodeError as error:
            self.get_logger().error(f"RIGHT: Json decode error when recv {data_right}: {error}")
            return
        except KeyError as error:
            self.get_logger().error(f"RIGHT: KeyError when recv {degree_data_right}: {error}")
            return
        except UnicodeDecodeError as error:
            self.get_logger().error(f"RIGHT: UnicodeDecodeError when recv {data_right}: {error}")
            return

        # Convert degree positions to radians
        radian_positions_left = [math.radians(deg) for deg in degree_positions_left]
        radian_positions_right = [math.radians(deg) for deg in degree_positions_right]

        # Publish the data to the serial_data topic
        # LEFT
        msg_left = JointState()
        msg_left.header.stamp = self.get_clock().now().to_msg()
        msg_left.name = self._joint_names
        msg_left.position = radian_positions_left
        msg_left.velocity = []
        msg_left.effort = []
        # RIGHT
        msg_right = copy.deepcopy(msg_left)
        msg_right.position = radian_positions_right

        self._left_publisher.publish(msg_left)
        self._right_publisher.publish(msg_right)
        current_time = self.get_clock().now()
        if current_time - self.last_log_time >= self.log_interval:
            self.get_logger().info(f'LEFT: Receive from arm esp32: {degree_data_left}')
            self.get_logger().info(f'RIGHT Receive from arm esp32: {degree_data_right}')
            self.last_log_time = current_time


def main(args=None):
    rclpy.init(args=args)

    serial_reader = ArmSerialReader()
    rclpy.spin(serial_reader)

    serial_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
