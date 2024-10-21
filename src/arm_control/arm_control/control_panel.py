import copy
import curses
import params
import utils


class ControlPanel():
    def __init__(self):
        # Robot control
        self._joint_cnt = params.DEFAULT_JOINT_NUMBER
        self._init_joint_left = params.DEFAULT_LEFT_JOINT_DEG_ANGLE
        self._init_joint_right = params.DEFAULT_RIGHT_JOINT_DEG_ANGLE
        self.cur_joint_left = copy.deepcopy(self._init_joint_left)
        self.cur_joint_right = copy.deepcopy(self._init_joint_right)

        # Control state
        self._cur_select_arm = self.cur_joint_left  # Start with the left joint array
        self._cur_select_joint = 0  # Start with the first element in cur_joint_left
        self._panel_state = utils.PanelState.NORMAL  # Start in normal mode
        self._panel_select = utils.PanelSelect.LEFT

    def reset_angle(self):
        self.cur_joint_left = copy.deepcopy(self._init_joint_left)
        self.cur_joint_right = copy.deepcopy(self._init_joint_right)
    
    def display(self, stdscr):
        stdscr.clear()
        # Mode
        if self._panel_state == utils.PanelState.NORMAL:
            stdscr.addstr(0, 0, f"Mode: Normal")
        elif self._panel_state == utils.PanelState.SELECT:
            stdscr.addstr(0, 0, f"Mode: Select")
        elif self._panel_state == utils.PanelState.CONTROL:
            stdscr.addstr(0, 0, f"Mode: Control")

        if self._panel_select == utils.PanelSelect.LEFT:
            stdscr.addstr(2, 0, f"cur_joint_left: {self.cur_joint_left}", curses.A_REVERSE)
            stdscr.addstr(3, 0, f"cur_joint_right: {self.cur_joint_right}")
            stdscr.addstr(5, 0, "Reset all angles")
        elif self._panel_select == utils.PanelSelect.RIGHT:
            stdscr.addstr(2, 0, f"cur_joint_left: {self.cur_joint_left}")
            stdscr.addstr(3, 0, f"cur_joint_right: {self.cur_joint_right}", curses.A_REVERSE)
            stdscr.addstr(5, 0, "Reset all angles")
        else:
            stdscr.addstr(2, 0, f"cur_joint_left: {self.cur_joint_left}")
            stdscr.addstr(3, 0, f"cur_joint_right: {self.cur_joint_right}")
            stdscr.addstr(5, 0, "Reset all angles", curses.A_REVERSE)
        
        if self._panel_state != utils.PanelState.NORMAL:
            # Highlight the active element in the current array
            for i, val in enumerate(self._cur_select_arm):
                if i == self._cur_select_joint:
                    if self._panel_state == utils.PanelState.SELECT:
                        stdscr.addstr(7 + i, 0, f"Joint {i + 1}: {val} <-", curses.A_REVERSE)
                    elif self._panel_state == utils.PanelState.CONTROL:
                        stdscr.addstr(7 + i, 0, f"-->> Joint {i + 1}: {val} <<--", curses.A_REVERSE)
                else:
                    stdscr.addstr(7 + i, 0, f"Joint {i + 1}: {val}")
        stdscr.refresh()

    def control_loop(self, stdscr):
        curses.curs_set(0)  # Hide the cursor
        
        while True:
            self.display(stdscr)
            key = stdscr.getch()

            if self._panel_state == utils.PanelState.NORMAL:
                if key == ord('q') or key == 27:  # ESC key
                    break  # Exit the control loop and script
                elif key == curses.KEY_UP:
                    self._panel_select = (self._panel_select - 1) % len(utils.PanelSelect)
                elif key == curses.KEY_DOWN:
                    self._panel_select = (self._panel_select + 1) % len(utils.PanelSelect)
                elif key == curses.KEY_ENTER or key in [10, 13]:
                    if self._panel_select == utils.PanelSelect.LEFT:
                        self._cur_select_arm = self.cur_joint_left
                        self._panel_state = utils.PanelState.SELECT
                    elif self._panel_select == utils.PanelSelect.RIGHT:
                        self._cur_select_arm = self.cur_joint_right
                        self._panel_state = utils.PanelState.SELECT
                    elif self._panel_select == utils.PanelSelect.RESET:
                        self.reset_angle()
                        self._panel_select = utils.PanelSelect.LEFT

            elif self._panel_state == utils.PanelState.SELECT:
                if key == curses.KEY_DOWN:
                    self._cur_select_joint = (self._cur_select_joint + 1) % len(self._cur_select_arm)
                elif key == curses.KEY_UP:
                    self._cur_select_joint = (self._cur_select_joint - 1) % len(self._cur_select_arm)
                elif key == curses.KEY_ENTER or key in [10, 13]:
                    self._panel_state = utils.PanelState.CONTROL
                elif key == ord('q') or key == 27:  # ESC key
                    self._panel_state = utils.PanelState.NORMAL

            elif self._panel_state == utils.PanelState.CONTROL:
                if key == curses.KEY_UP:
                    self._cur_select_arm[self._cur_select_joint] += 1  # Increase current array value
                elif key == curses.KEY_DOWN:
                    self._cur_select_arm[self._cur_select_joint] -= 1  # Decrease current array value
                elif key == ord('q') or key == 27:  # ESC key
                    self._panel_state = utils.PanelState.SELECT


def main(stdscr):
    panel = ControlPanel()
    panel.control_loop(stdscr)


if __name__ == "__main__":
    # https://stackoverflow.com/questions/27372068/why-does-the-escape-key-have-a-delay-in-python-curses?fbclid=IwZXh0bgNhZW0CMTEAAR1DwC-iF0EG7TBcISSEZL561OCsPDfn7mN524uqCH7TrN5Hp6qYVpLkTx0_aem_fN0t19jEhI1D6ZcEuJXYKA
    curses.set_escdelay(25)
    curses.wrapper(main)
