from arm_control.params import *
from enum import Enum, IntEnum
from typing import Type
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


KEY_MAP = {
    curses.KEY_UP: 0x00000001,
    curses.KEY_DOWN: 0x00000002,
    KEY_ESC: 0x00000004,
    ord('q'): 0x00000004,
    curses.KEY_ENTER: 0x00000008,
    10: 0x00000008,
    13: 0x00000008,
}


class ArmSide(IntEnum):
    """Declare the enum for LEFT and RIGHT arm
    The items must be sorted.
    The `ALL` bitmask is also included but is not meant to be set.
    """
    LEFT = 0x00000001
    RIGHT = 0x00000002
    ALL = 0xffffffff


class PanelState(IntEnum):
    """Declare the enum for state of control panel
    The items must be sorted.
    The `ALL` bitmask is also included but is not meant to be set.
    """
    EXIT = 0x00000001
    NORMAL = 0x00000002
    SELECT = 0x00000004
    CONTROL_JOINT = 0x00000008
    CONTROL_STEP = 0x000000010
    ALL = 0xffffffff


class PanelSelect(IntEnum):
    """Declare the enum for option of control panel
    The items must be sorted.
    The `ALL` bitmask is also included but is not meant to be set.
    """
    LEFT = 0x00000001
    RIGHT = 0x00000002
    STEP = 0x00000004
    RESET = 0x00000008
    ALL = 0xffffffff


def next_bitwise_enum(cur: IntEnum, enum_class: Type[IntEnum]) -> IntEnum:
    """Returns the next bitwise-based IntEnum item.

    This function provides a way to iterate through an `IntEnum` by shifting the current value
    bitwise. It wraps around if the next value exceeds the maximum value.

    Args:
        cur (IntEnum): The current enum item.
        enum_class (Type[IntEnum]): The class type of the enum being iterated.

    Returns:
        IntEnum: The next enum item, wrapping around if necessary.

    Example:
        >>> cur_sel = PanelSelect.LEFT
        >>> cur_sel = next_bitwise_enum(cur_sel, PanelSelect)
    """
    # Determine the shift based on the presence of `ALL` in the enum
    shift = len(enum_class) - \
        2 if 'ALL' in enum_class.__members__ else len(enum_class) - 1
    max_value = 1 << shift

    # Shift left to get the next item
    next_value = cur << 1

    # Wrap around if the shifted value exceeds the maximum
    if next_value > max_value:
        next_value = 1  # Start from the first item (bitwise 0x00000001)

    return enum_class(next_value)


def prev_bitwise_enum(cur: IntEnum, enum_class: Type[IntEnum]) -> IntEnum:
    """Returns the previous bitwise-based IntEnum item.

    This function provides a way to iterate backward through an `IntEnum` by shifting the current value
    bitwise. It wraps around if the previous value goes below 1.

    Args:
        cur (IntEnum): The current enum item.
        enum_class (Type[IntEnum]): The class type of the enum being iterated.

    Returns:
        IntEnum: The previous enum item, wrapping around if necessary.

    Example:
        >>> cur_sel = PanelSelect.LEFT
        >>> cur_sel = prev_bitwise_enum(cur_sel, PanelSelect)
    """
    # Determine the shift based on the presence of `ALL` in the enum
    shift = len(enum_class) - \
        2 if 'ALL' in enum_class.__members__ else len(enum_class) - 1
    max_value = 1 << shift

    # Shift left to get the previous item
    prev_value = cur >> 1

    # Wrap around if the shifted value exceeds the minimum (bitwise 0x00000001)
    if prev_value < 1:
        prev_value = max_value  # Back to the last item

    return enum_class(prev_value)


def key_trans(key: int) -> int:
    """Translate key codes for Enter and 'q' keys to standard codes.

    This function remaps different key codes for Enter (including 10 and 13) to `curses.KEY_ENTER`,
    and maps 'q' to `KEY_ESC`. Other key codes remain unchanged.

    Args:
        key (int): The key code input captured by `stdscr.getch()`.

    Returns:
        int: The translated key code.

    Example:
        >>> key = key_trans(10)  # Translates to curses.KEY_ENTER
        >>> key = key_trans(ord('q'))  # Translates to KEY_ESC
    """
    if key == curses.KEY_ENTER or key in [10, 13]:
        ret: int = KEY_ENTER
    elif key == ord('q') or key == KEY_ESC:
        ret: int = KEY_ESC
    else:
        ret: int = key
    return ret


if __name__ == "__main__":
    print("----- Begin Test for next_bitwise_enum and prev_bitwise_enum -----")
    print("--- ArmSide ---")
    _cur_arm = ArmSide.LEFT
    _cur_arm = prev_bitwise_enum(_cur_arm, ArmSide)
    print(ArmSide(_cur_arm).name)
    _cur_arm = next_bitwise_enum(_cur_arm, ArmSide)
    print(ArmSide(_cur_arm).name)
    print("--- PanelSelect ---")
    _cur_sel = PanelSelect.LEFT
    _cur_sel = prev_bitwise_enum(_cur_sel, PanelSelect)
    print(PanelSelect(_cur_sel).name)
    _cur_sel = next_bitwise_enum(_cur_sel, PanelSelect)
    print(PanelSelect(_cur_sel).name)
    print("--- PanelState ---")
    _cur_state = PanelState.EXIT
    _cur_state = prev_bitwise_enum(_cur_state, PanelState)
    print(PanelState(_cur_state).name)
    _cur_state = next_bitwise_enum(_cur_state, PanelState)
    print(PanelState(_cur_state).name)
    print("----- End Test for next_bitwise_enum and prev_bitwise_enum -----")
