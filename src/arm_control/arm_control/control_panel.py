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
        if bool(KEY_MAP[key] & KEY_MAP[curses.KEY_UP]):
            self.tui.select_prev_item()
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_DOWN]):
            self.tui.select_next_item()
        elif bool(KEY_MAP[key] & KEY_MAP[KEY_ENTER]):
            if bool(self.tui.cur_sel & PanelSelect.LEFT):
                self.tui.cur_sel_arm = self.tui.cur_joint_left
                self.tui.cur_sel_arm_min = MIN_LEFT_JOINT_DEG_ANGLE
                self.tui.cur_sel_arm_max = MAX_LEFT_JOINT_DEG_ANGLE
                self.tui.change_state(SelectJointState)
            elif bool(self.tui.cur_sel & PanelSelect.RIGHT):
                self.tui.cur_sel_arm = self.tui.cur_joint_right
                self.tui.cur_sel_arm_min = MIN_RIGHT_JOINT_DEG_ANGLE
                self.tui.cur_sel_arm_max = MAX_RIGHT_JOINT_DEG_ANGLE
                self.tui.change_state(SelectJointState)
            elif bool(self.tui.cur_sel & PanelSelect.PARAM):
                self.tui.change_state(SelectParamState)
            elif bool(self.tui.cur_sel & PanelSelect.RESET):
                self.tui.reset()
        elif bool(KEY_MAP[key] & KEY_MAP[KEY_ESC]):
            self.tui.change_state(ExitState)

    # override
    def render(self, stdscr):
        stdscr.addstr(0, 0, "Normal State")
        self.tui.render_panel(stdscr)


class ExitState(State):
    # override
    def handle_input(self, key):
        pass  # No interaction in exit state

    # override
    def render(self, stdscr):
        stdscr.addstr(0, 0, "Exiting...")


class SelectJointState(State):
    # override
    def handle_input(self, key):
        if bool(KEY_MAP[key] & KEY_MAP[KEY_ESC]):
            self.tui.change_state(NormalState)
        elif bool(KEY_MAP[key] & KEY_MAP[KEY_ENTER]):
            self.tui.change_state(ControlJointState)
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_UP]):
            self.tui.cur_sel_joint = (
                self.tui.cur_sel_joint - 1) % DEFAULT_JOINT_NUMBER
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_DOWN]):
            self.tui.cur_sel_joint = (
                self.tui.cur_sel_joint - 1) % DEFAULT_JOINT_NUMBER

    # override
    def render(self, stdscr):
        stdscr.addstr(0, 0, "Select Joint State")
        self.tui.render_panel(stdscr)


class SelectParamState(State):
    # override
    def handle_input(self, key):
        if bool(KEY_MAP[key] & KEY_MAP[KEY_ESC]):
            self.tui.change_state(NormalState)
        elif bool(KEY_MAP[key] & KEY_MAP[KEY_ENTER]):
            self.tui.change_state(ControlParamState)
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_UP]):
            self.tui.cur_sel_param = prev_bitwise_enum(
                self.tui.cur_sel_param, ParamSelect)
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_DOWN]):
            self.tui.cur_sel_param = next_bitwise_enum(
                self.tui.cur_sel_param, ParamSelect)

    # override
    def render(self, stdscr):
        stdscr.addstr(0, 0, "Select Param State")
        self.tui.render_panel(stdscr)


class ControlJointState(State):
    # override
    def handle_input(self, key):
        if bool(KEY_MAP[key] & KEY_MAP[KEY_ESC]):
            self.tui.change_state(SelectJointState)

    # override
    def render(self, stdscr):
        stdscr.addstr(0, 0, "Control Joint State")
        self.tui.render_panel(stdscr)


class ControlParamState(State):
    # override
    def handle_input(self, key):
        if bool(KEY_MAP[key] & KEY_MAP[KEY_ESC]):
            self.tui.change_state(SelectParamState)

    # override
    def render(self, stdscr):
        stdscr.addstr(0, 0, "Control Param State")
        self.tui.render_panel(stdscr)


##############################
############# TUI ############
##############################

class TUI:
    def __init__(self, arm_publisher_: ArmPublisher):
        self.state: State = NormalState(self)
        self.cur_sel: PanelSelect = PanelSelect.LEFT
        self.cur_sel_param: ParamSelect = ParamSelect.STEP
        self.cur_sel_joint: int = 0
        self.cur_joint_left: list = copy.deepcopy(DEFAULT_LEFT_JOINT_DEG_ANGLE)
        self.cur_joint_right: list = copy.deepcopy(
            DEFAULT_RIGHT_JOINT_DEG_ANGLE)

        # Start with the left joint array
        self._cur_sel_arm: list = self.cur_joint_left
        self._cur_sel_arm_min: list = MIN_LEFT_JOINT_DEG_ANGLE
        self._cur_sel_arm_max: list = MAX_LEFT_JOINT_DEG_ANGLE

        self.msg_cnt: int = 0
        self.key: int = 0

        self.param: list = [
            DEFAULT_JOINT_MOVE_STEP_DEG,
            DEFAULT_SPEED_DEG_PER_SEC,
            DEFAULT_DURATION_SEC,
            DEFAULT_FPS
        ]

        # Publisher for arm control
        self._arm_publisher = arm_publisher_

        # Publish the initial joint angles
        self._arm_publisher.pub_arm(
            self.cur_joint_left,
            self.cur_joint_right,
            thread_id=self.msg_cnt,
            show_info=False)
        self.msg_cnt += 1

        # Debug params
        self._key_not_define: bool = False
        self._trans_not_define: bool = False

    def select_prev_item(self):
        self.cur_sel = prev_bitwise_enum(self.cur_sel, PanelSelect)

    def select_next_item(self):
        self.cur_sel = next_bitwise_enum(self.cur_sel, PanelSelect)

    def change_state(self, new_state_cls: State):
        """Transition to a new state."""
        self.state = new_state_cls(self)

    def reset(self):
        """Reset to the initial state and item."""
        self.cur_sel = PanelSelect.LEFT
        self.change_state(NormalState)

    def render_panel(self, stdscr):
        """Render the panel with the current selection."""
        for i in range(get_len_bitwise_enum(PanelSelect)):
            item = 1 << i
            if item == self.cur_sel:
                stdscr.addstr(
                    i + 1, 0, f"> {PanelSelect(item).name}", curses.A_REVERSE)
            else:
                stdscr.addstr(i + 1, 0, f"  {PanelSelect(item).name}")

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
