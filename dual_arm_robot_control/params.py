import curses


# ROS control
ROS_QOS_DEPTH = 10
LEFT_ARM_TOPIC: str = "/left_arm"
RIGHT_ARM_TOPIC: str = "/right_arm"
LEFT_HAND_TOPIC: str = "/left_hand"
RIGHT_HAND_TOPIC: str = "/right_hand"

# Left arm angle
DEFAULT_LEFT_JOINT_DEG_ANGLE = [170, 10, 100, 10, 100, 100, 180, 0, 0, 0, 0]
MIN_LEFT_JOINT_DEG_ANGLE = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MAX_LEFT_JOINT_DEG_ANGLE = [180, 90, 180, 120, 180, 180, 180, 180, 180, 180, 180]

# Right arm angle
DEFAULT_RIGHT_JOINT_DEG_ANGLE = [10, 170, 80, 10, 80, 80, 0, 180, 180, 180, 180]
MIN_RIGHT_JOINT_DEG_ANGLE = [0, 80, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MAX_RIGHT_JOINT_DEG_ANGLE = [180, 180, 180, 120, 180, 180, 180, 180, 180, 180, 180]

ARM_BIAS: int = 0
HAND_BIAS: int = 6
DEFAULT_JOINT_NUMBER = len(DEFAULT_LEFT_JOINT_DEG_ANGLE)
DEFAULT_JOINT_MOVE_STEP_DEG = 5.0
MIN_JOINT_MOVE_STEP_DEG = 1
MAX_JOINT_MOVE_STEP_DEG = 50

# Use for monitor ESP32 states `arm_reader.py`
LEFT_JOINTS_STATE_TOPIC = "/left_arm_state"
RIGHT_JOINTS_STATE_TOPIC = "/right_arm_state"
READER_TIMER_PERIOD = 0.5  # seconds
READER_CALLBACK_TIMER_PERIOD = 0.1  # seconds

# Publish Joints with Speed
DEFAULT_SPEED_DEG_PER_SEC = 5.0
DEFAULT_DURATION_SEC = 0.1
DEFAULT_FPS = 10.0

# Key mapping
KEY_ESC: int = 27
KEY_BACKSPACE: int = 127
KEY_ENTER: int = curses.KEY_ENTER

# Curses settings
STATE_POS: int = 0
ERR_POS: int = 1
MENU_POS: int = 2
CONTROL_POS: int = 8
TEXT_INPUT_POS: int = 13


if DEFAULT_JOINT_NUMBER != len(DEFAULT_RIGHT_JOINT_DEG_ANGLE) or \
        DEFAULT_JOINT_NUMBER != len(MIN_LEFT_JOINT_DEG_ANGLE) or \
        DEFAULT_JOINT_NUMBER != len(MAX_LEFT_JOINT_DEG_ANGLE) or \
        DEFAULT_JOINT_NUMBER != len(MIN_RIGHT_JOINT_DEG_ANGLE) or \
        DEFAULT_JOINT_NUMBER != len(MAX_RIGHT_JOINT_DEG_ANGLE):
    raise ValueError(
        "Error, the number of joints is not consistent. Please check again.")
