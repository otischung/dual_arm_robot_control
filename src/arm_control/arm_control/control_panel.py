import copy
import curses
import rclpy
from arm_control.arm_publisher import *
from arm_control.params import *
from arm_control.utils import *


class ControlPanel():
    def __init__(self, arm_publisher_: ArmPublisher):
        # The transition table of the Finite State Machine
        self._transitions = [
            # Enter and Esc Keys
            {"src": PanelState.NORMAL, "dst": PanelState.NORMAL,
                "key": KEY_MAP[KEY_ENTER], "sel": PanelSelect.RESET, "exec": self.reset_angle},
            {"src": PanelState.NORMAL, "dst": PanelState.SELECT,
                "key": KEY_MAP[KEY_ENTER], "sel": PanelSelect.LEFT | PanelSelect.RIGHT, "exec": self._change_select_arm},
            {"src": PanelState.NORMAL, "dst": PanelState.CONTROL_STEP,
                "key": KEY_MAP[KEY_ENTER], "sel": PanelSelect.STEP, "exec": None},
            {"src": PanelState.NORMAL, "dst": PanelState.EXIT,
                "key": KEY_MAP[KEY_ESC], "sel": PanelSelect.ALL, "exec": self._quit_},
            {"src": PanelState.SELECT, "dst": PanelState.NORMAL,
                "key": KEY_MAP[KEY_ESC], "sel": PanelSelect.LEFT | PanelSelect.RIGHT, "exec": None},
            {"src": PanelState.SELECT, "dst": PanelState.CONTROL_JOINT,
                "key": KEY_MAP[KEY_ENTER], "sel": PanelSelect.LEFT | PanelSelect.RIGHT, "exec": None},
            {"src": PanelState.CONTROL_JOINT, "dst": PanelState.SELECT,
                "key": KEY_MAP[KEY_ESC], "sel": PanelSelect.LEFT | PanelSelect.RIGHT, "exec": None},
            {"src": PanelState.CONTROL_STEP, "dst": PanelState.NORMAL,
                "key": KEY_MAP[KEY_ESC], "sel": PanelSelect.STEP, "exec": None},
            # Up and Down Arrow Keys
            {"src": PanelState.NORMAL, "dst": PanelState.NORMAL,
                "key": KEY_MAP[curses.KEY_UP] | KEY_MAP[curses.KEY_DOWN], "sel": PanelSelect.ALL, "exec": self._change_select},
            {"src": PanelState.SELECT, "dst": PanelState.SELECT,
                "key": KEY_MAP[curses.KEY_UP] | KEY_MAP[curses.KEY_DOWN], "sel": PanelSelect.ALL, "exec": self._change_select_joint},
            {"src": PanelState.CONTROL_JOINT, "dst": PanelState.CONTROL_JOINT,
                "key": KEY_MAP[curses.KEY_UP] | KEY_MAP[curses.KEY_DOWN], "sel": PanelSelect.ALL, "exec": self._control_select_joint},
            {"src": PanelState.CONTROL_STEP, "dst": PanelState.CONTROL_STEP,
                "key": KEY_MAP[curses.KEY_UP] | KEY_MAP[curses.KEY_DOWN], "sel": PanelSelect.ALL, "exec": self._control_step},
        ]
        self._cur_state: PanelState = PanelState.NORMAL
        self._cur_sel: PanelSelect = PanelSelect.LEFT
        self._cur_sel_joint: int = 0
        self._move_step = DEFAULT_JOINT_MOVE_STEP_DEG
        self.cur_joint_left = copy.deepcopy(DEFAULT_LEFT_JOINT_DEG_ANGLE)
        self.cur_joint_right = copy.deepcopy(DEFAULT_RIGHT_JOINT_DEG_ANGLE)
        self._cur_sel_arm = self.cur_joint_left  # Start with the left joint array
        # Start with the left joint array
        self._cur_sel_arm_min = MIN_LEFT_JOINT_DEG_ANGLE
        # Start with the left joint array
        self._cur_sel_arm_max = MAX_LEFT_JOINT_DEG_ANGLE

        # Publisher for arm control
        self._arm_publisher = arm_publisher_
        self._arm_publisher.pub_arm(self.cur_joint_left, self.cur_joint_right)

    def _quit_(self, key: int = None):
        """Just a trigger function for state transition.
        """
        quit(0)

    def _change_select(self, key: int):
        if key == curses.KEY_UP:
            self._cur_sel = prev_bitwise_enum(self._cur_sel, PanelSelect)
        elif key == curses.KEY_DOWN:
            self._cur_sel = next_bitwise_enum(self._cur_sel, PanelSelect)
        else:
            raise KeyError(
                f"Error: The key {key} is not allowed in {self._change_select.__name__}")

    def _change_select_arm(self, key: int = None):
        if self._cur_sel == PanelSelect.LEFT:
            self._cur_sel_arm = self.cur_joint_left
            self._cur_sel_arm_min = MIN_LEFT_JOINT_DEG_ANGLE
            self._cur_sel_arm_max = MAX_LEFT_JOINT_DEG_ANGLE
        elif self._cur_sel == PanelSelect.RIGHT:
            self._cur_sel_arm = self.cur_joint_right
            self._cur_sel_arm_min = MIN_RIGHT_JOINT_DEG_ANGLE
            self._cur_sel_arm_max = MAX_RIGHT_JOINT_DEG_ANGLE
        else:
            raise KeyError(
                f"Error: The select mode {self._cur_sel.name} is not allowed in {self._change_select_arm.__name__}")

    def _change_select_joint(self, key: int):
        if key == curses.KEY_UP:
            self._cur_sel_joint = (
                self._cur_sel_joint - 1) % DEFAULT_JOINT_NUMBER
        elif key == curses.KEY_DOWN:
            self._cur_sel_joint = (
                self._cur_sel_joint + 1) % DEFAULT_JOINT_NUMBER
        else:
            raise KeyError(
                f"Error: The key {key} is not allowed in {self._change_select_joint.__name__}")

    def _control_select_joint(self, key: int):
        if key == curses.KEY_UP:
            self._cur_sel_arm[self._cur_sel_joint] = min(
                self._cur_sel_arm[self._cur_sel_joint] + self._move_step,
                self._cur_sel_arm_max[self._cur_sel_joint]
            )
            self._arm_publisher.pub_arm(
                self.cur_joint_left, self.cur_joint_right)
        elif key == curses.KEY_DOWN:
            self._cur_sel_arm[self._cur_sel_joint] = max(
                self._cur_sel_arm[self._cur_sel_joint] - self._move_step,
                self._cur_sel_arm_min[self._cur_sel_joint]
            )
            self._arm_publisher.pub_arm(
                self.cur_joint_left, self.cur_joint_right)
        else:
            raise KeyError(
                f"Error: The key {key} is not allowed in {self._control_select_joint.__name__}")

    def _control_step(self, key: int):
        if key == curses.KEY_UP:
            self._move_step = min(
                self._move_step + 1,
                MAX_JOINT_MOVE_STEP_DEG
            )
        elif key == curses.KEY_DOWN:
            self._move_step = max(
                self._move_step - 1,
                MIN_JOINT_MOVE_STEP_DEG
            )

    def reset_angle(self, key: int = None):
        self.cur_joint_left = copy.deepcopy(DEFAULT_LEFT_JOINT_DEG_ANGLE)
        self.cur_joint_right = copy.deepcopy(DEFAULT_RIGHT_JOINT_DEG_ANGLE)
        self._arm_publisher.pub_arm(self.cur_joint_left, self.cur_joint_right)
        self._cur_sel = PanelSelect.LEFT

    def control_loop(self, stdscr):
        while True:
            stdscr.clear()
            stdscr.addstr(1, 0, f"State:\t{self._cur_state.name}")
            stdscr.addstr(2, 0, f"Select:\t{self._cur_sel.name}")
            stdscr.addstr(3, 0, f"Select Joint:\t{self._cur_sel_joint}")
            stdscr.addstr(4, 0, f"Move Step:\t{self._move_step}")
            stdscr.addstr(5, 0, f"LEFT:\t{self.cur_joint_left}")
            stdscr.addstr(6, 0, f"RIGHT:\t{self.cur_joint_right}")
            key = stdscr.getch()
            key_bit = KEY_MAP[key]
            trans: dict = None

            for trans_ in self._transitions:
                # Check if match.
                if bool(trans_["key"] & key_bit) and \
                        bool(trans_["src"] & self._cur_state) and \
                        bool(trans_["sel"] & self._cur_sel):
                    trans = trans_
                    break

            if trans is None:
                stdscr.addstr(2, 0, "Error, there is NO matched transition.")
            else:
                if trans["exec"] is not None:
                    trans["exec"](key)
                else:
                    stdscr.addstr(
                        3, 0, f"Error, there is no execution function.")
                self._cur_state = trans["dst"]
            stdscr.refresh()


def curses_main(stdscr):
    arm_publisher_ = ArmPublisher()
    panel = ControlPanel(arm_publisher_)
    panel.control_loop(stdscr)


def main(args=None):
    # https://stackoverflow.com/questions/27372068/why-does-the-escape-key-have-a-delay-in-python-curses?fbclid=IwZXh0bgNhZW0CMTEAAR1DwC-iF0EG7TBcISSEZL561OCsPDfn7mN524uqCH7TrN5Hp6qYVpLkTx0_aem_fN0t19jEhI1D6ZcEuJXYKA
    rclpy.init(args=args)
    curses.set_escdelay(25)
    curses.wrapper(curses_main)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
