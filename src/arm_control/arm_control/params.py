### ROS control
ROS_QOS_DEPTH = 10
LEFT_JOINTS_TOPIC = "/left_arm"
RIGHT_JOINTS_TOPIC = "/right_arm"
DEFAULT_LEFT_JOINT_DEG_ANGLE = [170, 10, 80, 60, 80, 10, 10]
DEFAULT_RIGHT_JOINT_DEG_ANGLE = [10, 170, 80, 60, 80, 10, 10]
DEFAULT_JOINT_NUMBER = len(DEFAULT_LEFT_JOINT_DEG_ANGLE)

# Use for monitor ESP32 states `arm_reader.py`
LEFT_JOINTS_STATE_TOPIC = "/left_arm_state"
RIGHT_JOINTS_STATE_TOPIC = "/right_arm_state"
READER_TIMER_PERIOD = 0.5  # seconds
READER_CALLBACK_TIMER_PERIOD = 0.1  # seconds

### USB Serial
DEFAULT_LEFT_USB_SERIAL = "/dev/ttyACM0"
DEFAULT_RIGHT_USB_SERIAL = "/dev/ttyUSB0"
SERIAL_JSON_KEY = "servo_target_angles"  # Assuming this is the key in your JSON
BAUD_RATE = 115200
