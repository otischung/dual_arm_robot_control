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
        self.array = self.cur_joint_left  # Start with the left joint array
        self.index = 0  # Start with the first element in cur_joint_left
        self.is_left_array = True  # Track if controlling left array
        self.mode = utils.PanelState.NORMAL  # Start in normal mode
        self.select = utils.PanelSelect.LEFT

    def reset_angle(self):
        self.cur_joint_left = copy.deepcopy(self._init_joint_left)
        self.cur_joint_right = copy.deepcopy(self._init_joint_right)
    
    def display(self, stdscr):
        stdscr.clear()
        # Mode
        if self.mode == utils.PanelState.NORMAL:
            stdscr.addstr(0, 0, f"Mode: Normal")
        elif self.mode == utils.PanelState.SELECT:
            stdscr.addstr(0, 0, f"Mode: Select")
        elif self.mode == utils.PanelState.CONTROL:
            stdscr.addstr(0, 0, f"Mode: Control")

        if self.select == utils.PanelSelect.LEFT:
            stdscr.addstr(2, 0, f"cur_joint_left: {self.cur_joint_left}", curses.A_REVERSE)
            stdscr.addstr(3, 0, f"cur_joint_right: {self.cur_joint_right}")
            stdscr.addstr(5, 0, "Reset all angles")
        elif self.select == utils.PanelSelect.RIGHT:
            stdscr.addstr(2, 0, f"cur_joint_left: {self.cur_joint_left}")
            stdscr.addstr(3, 0, f"cur_joint_right: {self.cur_joint_right}", curses.A_REVERSE)
            stdscr.addstr(5, 0, "Reset all angles")
        else:
            stdscr.addstr(2, 0, f"cur_joint_left: {self.cur_joint_left}")
            stdscr.addstr(3, 0, f"cur_joint_right: {self.cur_joint_right}")
            stdscr.addstr(5, 0, "Reset all angles", curses.A_REVERSE)
        
        if self.mode != utils.PanelState.NORMAL:
            # Highlight the active element in the current array
            for i, val in enumerate(self.array):
                if i == self.index:
                    if self.mode == utils.PanelState.SELECT:
                        stdscr.addstr(7 + i, 0, f"Joint {i + 1}: {val} <-", curses.A_REVERSE)
                    elif self.mode == utils.PanelState.CONTROL:
                        stdscr.addstr(7 + i, 0, f"-->> Joint {i + 1}: {val} <<--", curses.A_REVERSE)
                else:
                    stdscr.addstr(7 + i, 0, f"Joint {i + 1}: {val}")
        stdscr.refresh()

    def control_loop(self, stdscr):
        curses.curs_set(0)  # Hide the cursor
        
        while True:
            self.display(stdscr)
            key = stdscr.getch()

            if self.mode == utils.PanelState.NORMAL:
                if key == ord('q') or key == 27:  # ESC key
                    break  # Exit the control loop and script
                elif key == curses.KEY_UP:
                    self.select = (self.select - 1) % len(utils.PanelSelect)
                elif key == curses.KEY_DOWN:
                    self.select = (self.select + 1) % len(utils.PanelSelect)
                elif key == curses.KEY_ENTER or key in [10, 13]:
                    if self.select == utils.PanelSelect.LEFT:
                        self.array = self.cur_joint_left
                        self.mode = utils.PanelState.SELECT
                    elif self.select == utils.PanelSelect.RIGHT:
                        self.array = self.cur_joint_right
                        self.mode = utils.PanelState.SELECT
                    elif self.select == utils.PanelSelect.RESET:
                        self.reset_angle()
                        self.select = utils.PanelSelect.LEFT

            elif self.mode == utils.PanelState.SELECT:
                if key == curses.KEY_DOWN:
                    self.index = (self.index + 1) % len(self.array)
                elif key == curses.KEY_UP:
                    self.index = (self.index - 1) % len(self.array)
                elif key == curses.KEY_ENTER or key in [10, 13]:
                    self.mode = utils.PanelState.CONTROL
                elif key == ord('q') or key == 27:  # ESC key
                    self.mode = utils.PanelState.NORMAL

            elif self.mode == utils.PanelState.CONTROL:
                if key == curses.KEY_UP:
                    self.array[self.index] += 1  # Increase current array value
                elif key == curses.KEY_DOWN:
                    self.array[self.index] -= 1  # Decrease current array value
                elif key == ord('q') or key == 27:  # ESC key
                    self.mode = utils.PanelState.SELECT


def main(stdscr):
    panel = ControlPanel()
    panel.control_loop(stdscr)


if __name__ == "__main__":
    # https://stackoverflow.com/questions/27372068/why-does-the-escape-key-have-a-delay-in-python-curses?fbclid=IwZXh0bgNhZW0CMTEAAR1DwC-iF0EG7TBcISSEZL561OCsPDfn7mN524uqCH7TrN5Hp6qYVpLkTx0_aem_fN0t19jEhI1D6ZcEuJXYKA
    curses.set_escdelay(25)
    curses.wrapper(main)
