import copy
import curses
import rclpy
from arm_control.arm_publisher import *
from arm_control.params import *
from arm_control.utils import *


class ControlPanel():
    def __init__(self, arm_publisher_: ArmPublisher):
        # Robot control
        self._joint_cnt = DEFAULT_JOINT_NUMBER
        self._init_joint_left = DEFAULT_LEFT_JOINT_DEG_ANGLE
        self._init_joint_right = DEFAULT_RIGHT_JOINT_DEG_ANGLE
        self._min_joint_left = MIN_LEFT_JOINT_DEG_ANGLE
        self._max_joint_left = MAX_LEFT_JOINT_DEG_ANGLE
        self._min_joint_right = MIN_RIGHT_JOINT_DEG_ANGLE
        self._max_joint_right = MAX_RIGHT_JOINT_DEG_ANGLE
        self.cur_joint_left = copy.deepcopy(self._init_joint_left)
        self.cur_joint_right = copy.deepcopy(self._init_joint_right)

        # Control state
        self._cur_select_arm = self.cur_joint_left  # Start with the left joint array
        self._cur_select_arm_min = self._min_joint_left  # Start with the left joint array
        self._cur_select_arm_max = self._max_joint_left  # Start with the left joint array
        self._cur_select_joint = 0  # Start with the first element in cur_joint_left
        self._panel_state = PanelState.NORMAL  # Start in normal mode
        self._panel_select = PanelSelect.LEFT

        # Publisher for arm control
        self._arm_publisher = arm_publisher_
        self._arm_publisher.pub_arm(self.cur_joint_left, self.cur_joint_right)

    def _display_mode(self, stdscr):
        if self._panel_state == PanelState.NORMAL:
            stdscr.addstr(0, 0, f"Mode: Normal")
        elif self._panel_state == PanelState.SELECT:
            stdscr.addstr(0, 0, f"Mode: Select")
        elif self._panel_state == PanelState.CONTROL:
            stdscr.addstr(0, 0, f"Mode: Control")
        else:
            raise NotImplementedError("Error: Unkown Panel State in Display Mode.")
    
    def _display_menu(self, stdscr):
        if self._panel_select == PanelSelect.LEFT:
            stdscr.addstr(2, 0, f"cur_joint_left: {self.cur_joint_left}", curses.A_REVERSE)
            stdscr.addstr(3, 0, f"cur_joint_right: {self.cur_joint_right}")
            stdscr.addstr(5, 0, "Reset all angles")
        elif self._panel_select == PanelSelect.RIGHT:
            stdscr.addstr(2, 0, f"cur_joint_left: {self.cur_joint_left}")
            stdscr.addstr(3, 0, f"cur_joint_right: {self.cur_joint_right}", curses.A_REVERSE)
            stdscr.addstr(5, 0, "Reset all angles")
        elif self._panel_select == PanelSelect.RESET:
            stdscr.addstr(2, 0, f"cur_joint_left: {self.cur_joint_left}")
            stdscr.addstr(3, 0, f"cur_joint_right: {self.cur_joint_right}")
            stdscr.addstr(5, 0, "Reset all angles", curses.A_REVERSE)
        else:
            raise NotImplementedError("Error: Unkown Panel Select State in Display Mode.")
    
    def _display_joint(self, stdscr):
        # Highlight the active element in the current array
        for i, val in enumerate(self._cur_select_arm):
            if i == self._cur_select_joint:
                if self._panel_state == PanelState.SELECT:
                    stdscr.addstr(7 + i, 0, f"Joint {i + 1}: {val} <-", curses.A_REVERSE)
                elif self._panel_state == PanelState.CONTROL:
                    stdscr.addstr(7 + i, 0, f"-->> Joint {i + 1}: {val} <<--", curses.A_REVERSE)
            else:
                stdscr.addstr(7 + i, 0, f"Joint {i + 1}: {val}")

    def _control_normal(self, key: int) -> bool:
        """The control function in normal mode.

        Args:
            key (int): The keyboard input caught by stdscr.getch()
        
        Returns:
            bool: Determine if the entire control script needs to exit or not.
                - True: Exit the script.
                - False: Continue running the script.
        
        Raises:
            NotImplementedError: If the script enters to an unknown Panel Select State.
        """
        if key == ord('q') or key == 27:  # ESC key
            return True  # Exit the control loop and script
        elif key == curses.KEY_UP:
            self._panel_select = (self._panel_select - 1) % len(PanelSelect)
        elif key == curses.KEY_DOWN:
            self._panel_select = (self._panel_select + 1) % len(PanelSelect)
        elif key == curses.KEY_ENTER or key in [10, 13]:
            if self._panel_select == PanelSelect.LEFT:
                self._cur_select_arm = self.cur_joint_left
                self._cur_select_arm_min = self._min_joint_left
                self._cur_select_arm_max = self._max_joint_left
                self._panel_state = PanelState.SELECT
            elif self._panel_select == PanelSelect.RIGHT:
                self._cur_select_arm = self.cur_joint_right
                self._cur_select_arm_min = self._min_joint_right
                self._cur_select_arm_max = self._max_joint_right
                self._panel_state = PanelState.SELECT
            elif self._panel_select == PanelSelect.RESET:
                self.reset_angle()
                self._panel_select = PanelSelect.LEFT
            else:
                raise NotImplementedError("Error: Unkown Panel Select State in Normal Control Mode.")
        else:
            pass  # Ignore the key
        return False
    
    def _control_select(self, key: int):
        if key == curses.KEY_DOWN:
            self._cur_select_joint = (self._cur_select_joint + 1) % len(self._cur_select_arm)
        elif key == curses.KEY_UP:
            self._cur_select_joint = (self._cur_select_joint - 1) % len(self._cur_select_arm)
        elif key == curses.KEY_ENTER or key in [10, 13]:
            self._panel_state = PanelState.CONTROL
        elif key == ord('q') or key == 27:  # ESC key
            self._panel_state = PanelState.NORMAL
        else:
            pass  # Ignore the key
    
    def _control_control(self, key: int):
        if key == curses.KEY_UP:
            self._cur_select_arm[self._cur_select_joint] = min(
                self._cur_select_arm[self._cur_select_joint] + DEFAULT_JOINT_MOVE_STEP_DEG,
                self._cur_select_arm_max[self._cur_select_joint]
            )
            self._arm_publisher.pub_arm(self.cur_joint_left, self.cur_joint_right)
        elif key == curses.KEY_DOWN:
            self._cur_select_arm[self._cur_select_joint] = max(
                self._cur_select_arm[self._cur_select_joint] - DEFAULT_JOINT_MOVE_STEP_DEG,
                self._cur_select_arm_min[self._cur_select_joint]
            )
            self._arm_publisher.pub_arm(self.cur_joint_left, self.cur_joint_right)
        elif key == ord('q') or key == 27:  # ESC key
            self._panel_state = PanelState.SELECT
        else:
            pass  # Ignore the key
    
    def _display(self, stdscr):
        stdscr.clear()
        self._display_mode(stdscr)
        self._display_menu(stdscr)
        if self._panel_state != PanelState.NORMAL:
            self._display_joint(stdscr)
        stdscr.refresh()
    
    def reset_angle(self):
        self.cur_joint_left = copy.deepcopy(self._init_joint_left)
        self.cur_joint_right = copy.deepcopy(self._init_joint_right)
        self._arm_publisher.pub_arm(self.cur_joint_left, self.cur_joint_right)

    def control_loop(self, stdscr):
        """The main control loop of ControlPanel()

        Raises:
            NotImplementedError: If the script enters to an unknown Panel State.
        """
        curses.curs_set(0)  # Hide the cursor
        
        while True:
            self._display(stdscr)
            key = stdscr.getch()

            if self._panel_state == PanelState.NORMAL:
                if self._control_normal(key):
                    break
            elif self._panel_state == PanelState.SELECT:
                self._control_select(key)
            elif self._panel_state == PanelState.CONTROL:
                self._control_control(key)
            else:
                raise NotImplementedError("Error: Unkown Panel State in the Control Loop.")


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
