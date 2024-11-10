from enum import Enum, IntEnum
from arm_control.params import *
import curses


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


# Declare the enum for LEFT and RIGHT arm
class ArmSide(IntEnum):
    LEFT = 0x00000001
    RIGHT = 0x00000002
    ALL = 0xffffffff


# Declare the enum for state of control panel
class PanelState(IntEnum):
    EXIT = 0x00000001
    NORMAL = 0x00000002
    SELECT = 0x00000004
    CONTROL_JOINT = 0x00000008
    CONTROL_STEP = 0x000000010
    ALL = 0xffffffff


# Declare the enum for option of control panel
class PanelSelect(IntEnum):
    LEFT = 0x00000001
    RIGHT = 0x00000002
    STEP = 0x00000004
    RESET = 0x00000008
    ALL = 0xffffffff


def key_trans(key: int) -> int:
    """This method aims to transfer all possible Enter key codes into curses.KEY_ENTER.
    And also match the key 'q' to key 'Esc'.
    Key remians unchanged if the key is not Enter key or key 'q'.

    Args:
        key(int): The keyboard input caught by stdscr.getch().
    
    Returns:
        ret(int): The transfered key code.
    """
    if key == curses.KEY_ENTER or key in [10, 13]:
        ret: int = KEY_ENTER
    elif key == ord('q') or key == KEY_ESC:
        ret: int = KEY_ESC
    else:
        ret: int = key
    return ret
