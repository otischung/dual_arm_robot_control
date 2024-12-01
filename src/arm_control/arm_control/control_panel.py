import copy
import curses
import rclpy
from abc import ABC, abstractmethod
from arm_control.arm_publisher import *
from arm_control.params import *
from arm_control.utils import *

# A circular dependency exists between the TUI and the State class,
# requiring them to be implemented together in a single file.


##############################
########### States ###########
##############################

# Abstract Base Class for States
class State(ABC):
    def __init__(self, tui):
        self.tui = tui

    @abstractmethod
    def handle_input(self, key: int):
        """Handle keyboard input for the current state."""
        pass

    @abstractmethod
    def render(self, stdscr):
        """Render the screen for the current state."""
        pass


# Concrete States
class NormalState(State):
    # override. The @override decorator is only supported in Python 3.12+
    def handle_input(self, key: int):
        if key not in KEY_MAP:
            return

        if bool(KEY_MAP[key] & KEY_MAP[curses.KEY_UP]):
            self.tui.select_prev_item()
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_DOWN]):
            self.tui.select_next_item()
        elif bool(KEY_MAP[key] & KEY_MAP[KEY_ENTER]):
            if bool(self.tui.cur_sel & PanelSelect.LEFT):
                self.tui.set_to_left()
                self.tui.change_state(SelectJointState)
            elif bool(self.tui.cur_sel & PanelSelect.RIGHT):
                self.tui.set_to_right()
                self.tui.change_state(SelectJointState)
            elif bool(self.tui.cur_sel & PanelSelect.PARAM):
                self.tui.change_state(SelectParamState)
            elif bool(self.tui.cur_sel & PanelSelect.RESET_ANGLE):
                self.tui.reset_angle()
            elif bool(self.tui.cur_sel & PanelSelect.RESET_PARAM):
                self.tui.reset_param()
        elif bool(KEY_MAP[key] & KEY_MAP[KEY_ESC]):
            self.tui.change_state(ExitState)

    # override
    def render(self, stdscr):
        self.tui.display_single(stdscr, STATE_POS, "Panel State: Normal State")
        self.tui.display_menu(stdscr)


class ExitState(State):
    # override
    def handle_input(self, key):
        pass  # No interaction in exit state

    # override
    def render(self, stdscr):
        pass


class SelectJointState(State):
    # override
    def handle_input(self, key):
        if key not in KEY_MAP:
            return

        if bool(KEY_MAP[key] & KEY_MAP[KEY_ESC]):
            self.tui.change_state(NormalState)
        elif bool(KEY_MAP[key] & KEY_MAP[KEY_ENTER]):
            self.tui.change_state(ControlJointState)
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_UP]):
            self.tui.select_prev_joint()
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_DOWN]):
            self.tui.select_next_joint()

    # override
    def render(self, stdscr):
        self.tui.display_single(stdscr, STATE_POS, "Panel State: Select Joint State")
        self.tui.display_menu(stdscr)
        self.tui.display_sel_joint(stdscr)


class SelectParamState(State):
    # override
    def handle_input(self, key):
        if key not in KEY_MAP:
            return

        if bool(KEY_MAP[key] & KEY_MAP[KEY_ESC]):
            self.tui.change_state(NormalState)
        elif bool(KEY_MAP[key] & KEY_MAP[KEY_ENTER]):
            if self.tui.cur_sel_param == ParamSelect.MODE:
                self.tui.change_state(ControlParamModeState)
            else:
                self.tui.change_state(ControlParamState)
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_UP]):
            self.tui.select_prev_param()
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_DOWN]):
            self.tui.select_next_param()

    # override
    def render(self, stdscr):
        self.tui.display_single(stdscr, STATE_POS, "Panel State: Select Param State")
        self.tui.display_menu(stdscr)
        self.tui.display_sel_param(stdscr)


class ControlJointState(State):
    # override
    def handle_input(self, key):
        if key not in KEY_MAP:
            return

        if bool(KEY_MAP[key] & KEY_MAP[KEY_ESC]):
            self.tui.change_state(SelectJointState)
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_UP]):
            self.tui.increase_joint()
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_DOWN]):
            self.tui.decrease_joint()

    # override
    def render(self, stdscr):
        self.tui.display_single(stdscr, STATE_POS, "Panel State: Control Joint State")
        self.tui.display_menu(stdscr)
        self.tui.display_control_joint(stdscr)


class ControlParamModeState(State):
    # override
    def handle_input(self, key):
        if key not in KEY_MAP:
            return

        if bool(KEY_MAP[key] & KEY_MAP[KEY_ESC]):
            self.tui.change_state(SelectParamState)
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_UP]):
            self.tui.select_prev_mode()
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_DOWN]):
            self.tui.select_next_mode()

    # override
    def render(self, stdscr):
        self.tui.display_single(stdscr, STATE_POS, "Panel State: Control Param Mode State")
        self.tui.display_menu(stdscr)
        self.tui.display_control_param(stdscr)


class ControlParamState(State):
    def __init__(self, tui):
        super().__init__(tui)
        self.input_string = ""  # Stores the user input

    # override
    def handle_input(self, key):
        error: bool = False

        # We will get keyboard input here and thus we don't use the KEY_MAP dictionary.
        if key == KEY_ESC or key in [curses.KEY_ENTER, 10, 13]:
            try:
                value = float(self.input_string)
            except Exception as e:
                error = True

            if not error:
                if self.tui.cur_sel_param == ParamSelect.STEP:
                    self.tui.step = value
                elif self.tui.cur_sel_param == ParamSelect.PARAM:
                    self.tui.param = value
                elif self.tui.cur_sel_param == ParamSelect.FPS:
                    self.tui.fps = value

            self.tui.change_state(SelectParamState)

        elif key in range(32, 127):  # ASCII printable characters
            self.input_string += chr(key)  # Add character to the input string
        elif key == curses.KEY_BACKSPACE or key == KEY_BACKSPACE:  # Handle backspace
            self.input_string = self.input_string[:-1]  # Remove last character

    # override
    def render(self, stdscr):
        self.tui.display_single(stdscr, STATE_POS, "Panel State: Control Param State")
        self.tui.display_menu(stdscr)
        self.tui.display_control_param(stdscr)

        # Render the current input string
        self.tui.display_single(stdscr, TEXT_INPUT_POS, f"Input: {self.input_string}")


##############################
############# TUI ############
##############################

class TUI:
    def __init__(self, arm_publisher_: ArmPublisher):
        self.state: State = NormalState(self)
        self.key: int = 0

        # Arm settings
        self.cur_sel: PanelSelect = PanelSelect.LEFT
        self.cur_sel_param: ParamSelect = ParamSelect.STEP
        self.cur_sel_joint: int = 0
        self.cur_joint_left: list = copy.deepcopy(DEFAULT_LEFT_JOINT_DEG_ANGLE)
        self.cur_joint_right: list = copy.deepcopy(
            DEFAULT_RIGHT_JOINT_DEG_ANGLE)
        self.cur_sel_arm: list = self.cur_joint_left
        self.cur_sel_arm_min: list = MIN_LEFT_JOINT_DEG_ANGLE
        self.cur_sel_arm_max: list = MAX_LEFT_JOINT_DEG_ANGLE

        # Parameter settings
        self.step: float = DEFAULT_JOINT_MOVE_STEP_DEG
        self.param_mode: ParamMode = ParamMode.SPEED
        self.param: float = DEFAULT_SPEED_DEG_PER_SEC
        self.fps: float = DEFAULT_FPS

        # Publisher for arm control
        self.arm_publisher = arm_publisher_
        self.msg_cnt: int = 0

        # Publish the initial joint angles
        self.arm_publisher.pub_arm(
            self.cur_joint_left,
            self.cur_joint_right,
            thread_id=self.msg_cnt,
            show_info=False)
        self.msg_cnt += 1

    def display_single(self, stdscr, y_position: int, message: str, is_highlight: bool = False):
        style: int = curses.A_REVERSE if is_highlight else 0
        stdscr.addstr(y_position, 0, message, style)

    def display_menu(self, stdscr, y_position: int = MENU_POS):
        self.display_single(
            stdscr, y_position, f"LEFT arm: {self.cur_joint_left}", self.cur_sel == PanelSelect.LEFT)
        self.display_single(
            stdscr, y_position + 1, f"RIGHT arm: {self.cur_joint_right}", self.cur_sel == PanelSelect.RIGHT)
        self.display_single(
            stdscr, y_position + 2, f"Step (deg): {self.step}, {self.param_mode.name}: {self.param}, fps: {self.fps}", self.cur_sel == PanelSelect.PARAM)
        self.display_single(
            stdscr, y_position + 3, "Reset all angles", self.cur_sel == PanelSelect.RESET_ANGLE)
        self.display_single(
            stdscr, y_position + 4, "Reset all parameters", self.cur_sel == PanelSelect.RESET_PARAM)

    def display_sel_joint(self, stdscr, y_position: int = CONTROL_POS):
        # Highlight the active element in the current array
        for i, val in enumerate(self.cur_sel_arm):
            self.display_single(
                stdscr, y_position + i, f"Joint {i + 1}: {val}", i == self.cur_sel_joint)

    def display_control_joint(self, stdscr, y_position: int = CONTROL_POS):
        # Highlight the active element in the current array
        for i, val in enumerate(self.cur_sel_arm):
            if i == self.cur_sel_joint:
                self.display_single(
                    stdscr, y_position + i, f"--> Joint {i + 1}: {val} <--", True)
            else:
                self.display_single(
                    stdscr, y_position + i, f"Joint {i + 1}: {val}", False)

    def display_sel_param(self, stdscr, y_position: int = CONTROL_POS):
        self.display_single(
            stdscr, y_position, f"Step: {self.step}", self.cur_sel_param == ParamSelect.STEP)
        self.display_single(
            stdscr, y_position + 1, f"Mode: {self.param_mode.name}", self.cur_sel_param == ParamSelect.MODE)
        self.display_single(
            stdscr, y_position + 2, f"{self.param_mode.name}: {self.param}", self.cur_sel_param == ParamSelect.PARAM)
        self.display_single(
            stdscr, y_position + 3, f"FPS: {self.fps}", self.cur_sel_param == ParamSelect.FPS)

    def display_control_param(self, stdscr, y_position: int = CONTROL_POS):
        if self.cur_sel_param == ParamSelect.STEP:
            self.display_single(stdscr, y_position, f"--> Step: {self.step} <--", True)
        else:
            self.display_single(stdscr, y_position, f"Step: {self.step}", False)
        if self.cur_sel_param == ParamSelect.MODE:
            self.display_single(stdscr, y_position + 1, f"--> Mode: {self.param_mode.name} <--", True)
        else:
            self.display_single(stdscr, y_position + 1, f"Mode: {self.param_mode.name}", False)
        if self.cur_sel_param == ParamSelect.PARAM:
            self.display_single(stdscr, y_position + 2, f"--> {self.param_mode.name}: {self.param} <--", True)
        else:
            self.display_single(stdscr, y_position + 2, f"{self.param_mode.name}: {self.param}", False)
        if self.cur_sel_param == ParamSelect.FPS:
            self.display_single(stdscr, y_position + 3, f"--> FPS: {self.fps} <--", True)
        else:
            self.display_single(stdscr, y_position + 3, f"FPS: {self.fps}", False)

    def select_prev_item(self):
        self.cur_sel = prev_bitwise_enum(self.cur_sel, PanelSelect)

    def select_next_item(self):
        self.cur_sel = next_bitwise_enum(self.cur_sel, PanelSelect)

    def change_state(self, new_state_cls: State):
        """Transition to a new state."""
        self.state = new_state_cls(self)

    def set_to_left(self):
        self.cur_sel_arm = self.cur_joint_left
        self.cur_sel_arm_min = MIN_LEFT_JOINT_DEG_ANGLE
        self.cur_sel_arm_max = MAX_LEFT_JOINT_DEG_ANGLE

    def set_to_right(self):
        self.cur_sel_arm = self.cur_joint_right
        self.cur_sel_arm_min = MIN_RIGHT_JOINT_DEG_ANGLE
        self.cur_sel_arm_max = MAX_RIGHT_JOINT_DEG_ANGLE

    def select_prev_joint(self):
        self.cur_sel_joint = (
            self.cur_sel_joint - 1) % DEFAULT_JOINT_NUMBER

    def select_next_joint(self):
        self.cur_sel_joint = (
            self.cur_sel_joint + 1) % DEFAULT_JOINT_NUMBER
    
    def increase_joint(self):
        self.cur_sel_arm[self.cur_sel_joint] = min(
            self.cur_sel_arm[self.cur_sel_joint] + self.step,
            self.cur_sel_arm_max[self.cur_sel_joint]
        )
    
    def decrease_joint(self):
        self.cur_sel_arm[self.cur_sel_joint] = max(
            self.cur_sel_arm[self.cur_sel_joint] - self.step,
            self.cur_sel_arm_min[self.cur_sel_joint]
        )
    
    def select_prev_param(self):
        self.cur_sel_param = prev_bitwise_enum(self.cur_sel_param, ParamSelect)
    
    def select_next_param(self):
        self.cur_sel_param = next_bitwise_enum(self.cur_sel_param, ParamSelect)

    def select_prev_mode(self):
        self.param_mode = prev_bitwise_enum(self.param_mode, ParamMode)
    
    def select_next_mode(self):
        self.param_mode = next_bitwise_enum(self.param_mode, ParamMode)

    def reset_angle(self):
        """Reset to the initial state and item."""
        self.cur_sel: PanelSelect = PanelSelect.LEFT
        self.set_to_left()
        self.cur_sel_param: ParamSelect = ParamSelect.STEP
        self.cur_sel_joint: int = 0
        self.cur_joint_left: list = copy.deepcopy(DEFAULT_LEFT_JOINT_DEG_ANGLE)
        self.cur_joint_right: list = copy.deepcopy(
            DEFAULT_RIGHT_JOINT_DEG_ANGLE)

    def reset_param(self):
        """Reset to the initial parameter value and mode."""
        self.cur_sel: PanelSelect = PanelSelect.LEFT
        self.set_to_left()
        self.step: float = DEFAULT_JOINT_MOVE_STEP_DEG
        self.param_mode: ParamMode = ParamMode.SPEED
        self.param: float = DEFAULT_SPEED_DEG_PER_SEC
        self.fps: float = DEFAULT_FPS

    def run(self, stdscr):
        curses.curs_set(0)
        stdscr.clear()
        while not isinstance(self.state, ExitState):
            stdscr.clear()
            self.state.render(stdscr)
            stdscr.refresh()
            key = stdscr.getch()
            self.state.handle_input(key)


def curses_main(stdscr, arm_publisher: ArmPublisher):
    panel = TUI(arm_publisher)
    panel.run(stdscr)


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
