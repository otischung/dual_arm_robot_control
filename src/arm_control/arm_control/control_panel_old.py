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
                "key": KEY_MAP[KEY_ENTER], "sel": PanelSelect.RESET_ANGLE, "exec": self.reset_angle},
            {"src": PanelState.NORMAL, "dst": PanelState.SELECT_JOINT,
                "key": KEY_MAP[KEY_ENTER], "sel": PanelSelect.LEFT | PanelSelect.RIGHT, "exec": self._change_select_arm},
            {"src": PanelState.NORMAL, "dst": PanelState.SELECT_PARAM,
                "key": KEY_MAP[KEY_ENTER], "sel": PanelSelect.PARAM, "exec": None},
            {"src": PanelState.NORMAL, "dst": PanelState.EXIT,
                "key": KEY_MAP[KEY_ESC], "sel": PanelSelect.ALL, "exec": self._quit_},
            {"src": PanelState.SELECT_JOINT, "dst": PanelState.NORMAL,
                "key": KEY_MAP[KEY_ESC], "sel": PanelSelect.LEFT | PanelSelect.RIGHT, "exec": None},
            {"src": PanelState.SELECT_JOINT, "dst": PanelState.CONTROL_JOINT,
                "key": KEY_MAP[KEY_ENTER], "sel": PanelSelect.LEFT | PanelSelect.RIGHT, "exec": None},
            {"src": PanelState.CONTROL_JOINT, "dst": PanelState.SELECT_JOINT,
                "key": KEY_MAP[KEY_ESC], "sel": PanelSelect.LEFT | PanelSelect.RIGHT, "exec": None},
            {"src": PanelState.SELECT_PARAM, "dst": PanelState.NORMAL,
                "key": KEY_MAP[KEY_ESC], "sel": PanelSelect.PARAM, "exec": None},
            {"src": PanelState.SELECT_PARAM, "dst": PanelState.CONTROL_PARAM,
                "key": KEY_MAP[KEY_ENTER], "sel": PanelSelect.PARAM, "exec": None},
            {"src": PanelState.CONTROL_PARAM, "dst": PanelState.SELECT_PARAM,
                "key": KEY_MAP[KEY_ESC], "sel": PanelSelect.PARAM, "exec": None},
            # Up and Down Arrow Keys
            {"src": PanelState.NORMAL, "dst": PanelState.NORMAL,
                "key": KEY_MAP[curses.KEY_UP] | KEY_MAP[curses.KEY_DOWN], "sel": PanelSelect.ALL, "exec": self._change_select},
            {"src": PanelState.SELECT_JOINT, "dst": PanelState.SELECT_JOINT,
                "key": KEY_MAP[curses.KEY_UP] | KEY_MAP[curses.KEY_DOWN], "sel": PanelSelect.ALL, "exec": self._change_select_joint},
            {"src": PanelState.CONTROL_JOINT, "dst": PanelState.CONTROL_JOINT,
                "key": KEY_MAP[curses.KEY_UP] | KEY_MAP[curses.KEY_DOWN], "sel": PanelSelect.ALL, "exec": self._control_select_joint},
            {"src": PanelState.SELECT_PARAM, "dst": PanelState.SELECT_PARAM,
                "key": KEY_MAP[curses.KEY_UP] | KEY_MAP[curses.KEY_DOWN], "sel": PanelSelect.ALL, "exec": self._change_select_param},
            {"src": PanelState.CONTROL_PARAM, "dst": PanelState.CONTROL_PARAM,
                "key": KEY_MAP[curses.KEY_UP] | KEY_MAP[curses.KEY_DOWN], "sel": PanelSelect.ALL, "exec": self._control_param},
        ]
        self._key_not_define: bool = False
        self._trans_not_define: bool = False
        self._msg_cnt: int = 0
        self.key: int = 0
        self._cur_state: PanelState = PanelState.NORMAL
        self._cur_sel: PanelSelect = PanelSelect.LEFT
        self._cur_sel_param: ParamSelect = ParamSelect.STEP
        self._cur_sel_joint: int = 0
        self.cur_joint_left = copy.deepcopy(DEFAULT_LEFT_JOINT_DEG_ANGLE)
        self.cur_joint_right = copy.deepcopy(DEFAULT_RIGHT_JOINT_DEG_ANGLE)

        # Parameters
        # self._move_step: int = DEFAULT_JOINT_MOVE_STEP_DEG
        # self._speed: float = DEFAULT_SPEED_DEG_PER_SEC
        # self._duration: float = DEFAULT_DURATION_SEC
        # self._fps: float = DEFAULT_FPS
        self._param: list = [
            DEFAULT_JOINT_MOVE_STEP_DEG,
            DEFAULT_SPEED_DEG_PER_SEC,
            DEFAULT_DURATION_SEC,
            DEFAULT_FPS
        ]

        # Start with the left joint array
        self._cur_sel_arm = self.cur_joint_left
        self._cur_sel_arm_min = MIN_LEFT_JOINT_DEG_ANGLE
        self._cur_sel_arm_max = MAX_LEFT_JOINT_DEG_ANGLE

        # Publisher for arm control
        self._arm_publisher = arm_publisher_
        self._arm_publisher.pub_arm(
            self.cur_joint_left,
            self.cur_joint_right,
            thread_id=self._msg_cnt,
            show_info=False)
        self._msg_cnt += 1

    def _display_single(self, stdscr, message: str, y_position: int, is_highlight: bool = False):
        style = curses.A_REVERSE if is_highlight else 0
        stdscr.addstr(y_position, 0, message, style)

    def _display_menu(self, stdscr, y_position: int = 2):
        self._display_single(
            stdscr, f"LEFT arm: {self.cur_joint_left}", y_position + 0, self._cur_sel == PanelSelect.LEFT)
        self._display_single(
            stdscr, f"RIGHT arm: {self.cur_joint_right}", y_position + 1, self._cur_sel == PanelSelect.RIGHT)
        self._display_single(
            stdscr, f"Step (deg): {self._param[0]}, speed (deg/s): {self._param[1]}, duration (sec): {self._param[2]}, fps: {self._param[3]}", y_position + 2, self._cur_sel == PanelSelect.PARAM)
        self._display_single(
            stdscr, "Reset all angles", y_position + 3, self._cur_sel == PanelSelect.RESET_ANGLE)

    def _display_control_joint(self, stdscr, y_position: int = 7):
        # Highlight the active element in the current array
        for i, val in enumerate(self._cur_sel_arm):
            if self._cur_state == PanelState.CONTROL_JOINT and i == self._cur_sel_joint:
                message = f"-->> Joint {i + 1}: {val} <<--"
            else:
                message = f"Joint {i + 1}: {val}"
            self._display_single(
                stdscr, message, y_position + i, i == self._cur_sel_joint)

    def _display_control_param(self, stdscr, y_position: int = 7):
        for i in range(get_len_bitwise_enum(ParamSelect)):
            bitwise_id = 1 << i
            param_sel_it = ParamSelect(bitwise_id)
            if self._cur_state == PanelState.CONTROL_PARAM and self._cur_sel_param == bitwise_id:
                step_msg = f"-->> {param_sel_it.name}: {self._param[i]} <<--"
            else:
                step_msg = f"{param_sel_it.name}: {self._param[i]}"
            self._display_single(
                stdscr, step_msg, y_position + i, self._cur_sel_param == bitwise_id)

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

    def _change_select_param(self, key: int):
        if key == curses.KEY_UP:
            self._cur_sel_param = prev_bitwise_enum(
                self._cur_sel_param, ParamSelect)
        elif key == curses.KEY_DOWN:
            self._cur_sel_param = next_bitwise_enum(
                self._cur_sel_param, ParamSelect)
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
                self._cur_sel_arm[self._cur_sel_joint] + self._param[0],
                self._cur_sel_arm_max[self._cur_sel_joint]
            )
            self._arm_publisher.pub_arm(
                self.cur_joint_left,
                self.cur_joint_right,
                thread_id=self._msg_cnt,
                show_info=False)
            self._msg_cnt += 1
        elif key == curses.KEY_DOWN:
            self._cur_sel_arm[self._cur_sel_joint] = max(
                self._cur_sel_arm[self._cur_sel_joint] - self._param[0],
                self._cur_sel_arm_min[self._cur_sel_joint]
            )
            self._arm_publisher.pub_arm(
                self.cur_joint_left,
                self.cur_joint_right,
                thread_id=self._msg_cnt,
                show_info=False)
            self._msg_cnt += 1
        else:
            raise KeyError(
                f"Error: The key {key} is not allowed in {self._control_select_joint.__name__}")

    def _control_param(self, key: int):
        """WARNING: This function has bugs.
        
        This function use the constraints of the step to other parameters.
        However, the speed, duration and FPS should be floating points.
        If you want to fix this, you need to let user input the value directly.
        """
        if key == curses.KEY_UP:
            self._param[count_trailing_zeros_bitwise(self._cur_sel_param)] = min(
                self._param[count_trailing_zeros_bitwise(self._cur_sel_param)] + 1,
                MAX_JOINT_MOVE_STEP_DEG
            )
        elif key == curses.KEY_DOWN:
            self._param[count_trailing_zeros_bitwise(self._cur_sel_param)] = max(
                self._param[count_trailing_zeros_bitwise(self._cur_sel_param)] - 1,
                MIN_JOINT_MOVE_STEP_DEG
            )
        else:
            raise KeyError(
                f"Error: The key {key} is not allowed in {self._control_param.__name__}")

    def reset_angle(self, key: int = None):
        self.cur_joint_left = copy.deepcopy(DEFAULT_LEFT_JOINT_DEG_ANGLE)
        self.cur_joint_right = copy.deepcopy(DEFAULT_RIGHT_JOINT_DEG_ANGLE)
        self._arm_publisher.pub_arm(
            self.cur_joint_left,
            self.cur_joint_right,
            thread_id=self._msg_cnt,
            show_info=False)
        self._msg_cnt += 1
        self._cur_sel = PanelSelect.LEFT

    def control_loop(self, stdscr):
        while True:
            stdscr.clear()

            # Current state
            self._display_single(
                stdscr, f"Panel State: {self._cur_state.name}", 0)

            # Error message
            if self._key_not_define:
                self._display_single(
                    stdscr, f"Warning: The key {self.key} is not defined, ignoring.", 1)
            elif self._trans_not_define:
                self._display_single(
                    stdscr, f"Warning: No transtition for key {self.key} in current state, ignoring.", 1)

            # Menu
            self._display_menu(stdscr)

            # Control
            if bool(self._cur_state & (PanelState.SELECT_JOINT | PanelState.CONTROL_JOINT)):
                self._display_control_joint(stdscr)
            elif bool(self._cur_state & (PanelState.SELECT_PARAM | PanelState.CONTROL_PARAM)):
                self._display_control_param(stdscr)

            self._key_not_define = False
            self._trans_not_define = False
            self.key = stdscr.getch()
            try:
                key_bit = KEY_MAP[self.key]
            except KeyError:
                self._key_not_define = True
                continue
            trans: dict = None

            for trans_ in self._transitions:
                # Check if match.
                if bool(trans_["key"] & key_bit) and \
                        bool(trans_["src"] & self._cur_state) and \
                        bool(trans_["sel"] & self._cur_sel):
                    trans = trans_
                    break

            # Check if there matches any transition rules.
            if trans is None:
                self._trans_not_define = True
                stdscr.refresh()
                time.sleep(0.1)
                continue
            else:
                # Check if the transition rule needs to execute the desired function.
                if trans["exec"] is not None:
                    trans["exec"](self.key)

                # Move to the target state.
                self._cur_state = trans["dst"]

            stdscr.refresh()


def curses_main(stdscr, arm_publisher: ArmPublisher):
    panel = ControlPanel(arm_publisher)
    panel.control_loop(stdscr)


def main(args=None):
    rclpy.init(args=args)

    # Create the ROS 2 node
    arm_publisher = ArmPublisher()

    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(arm_publisher)

    # Run the executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        # Set curses ESC delay and run the curses interface
        # Reference: https://stackoverflow.com/questions/27372068/why-does-the-escape-key-have-a-delay-in-python-curses?fbclid=IwZXh0bgNhZW0CMTEAAR1DwC-iF0EG7TBcISSEZL561OCsPDfn7mN524uqCH7TrN5Hp6qYVpLkTx0_aem_fN0t19jEhI1D6ZcEuJXYKA
        curses.set_escdelay(25)
        curses.wrapper(curses_main, arm_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up on exit
        arm_publisher.destroy_node()
        executor.shutdown()
        executor_thread.join()
        rclpy.shutdown()


if __name__ == "__main__":
    main()