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

    def switch_array(self):
        if self.is_left_array:
            self.array = self.cur_joint_right
        else:
            self.array = self.cur_joint_left
        self.is_left_array = not self.is_left_array
    
    def display(self, stdscr):
        stdscr.clear()
        stdscr.addstr(0, 0, f"Mode: {self.mode}")
        stdscr.addstr(2, 0, "Current Array: " + ("Left Joint" if self.is_left_array else "Right Joint"))
        if self.is_left_array:
            stdscr.addstr(4, 0, f"cur_joint_left: {self.cur_joint_left}", curses.A_REVERSE)
            stdscr.addstr(5, 0, f"cur_joint_right: {self.cur_joint_right}")
        else:
            stdscr.addstr(4, 0, f"cur_joint_left: {self.cur_joint_left}")
            stdscr.addstr(5, 0, f"cur_joint_right: {self.cur_joint_right}", curses.A_REVERSE)
        stdscr.addstr(7, 0, f"Control Index: {self.index}")
        
        # Highlight the active element in the current array
        for i, val in enumerate(self.array):
            if i == self.index:
                stdscr.addstr(9 + i, 0, f"Element {i}: {val} <-", curses.A_REVERSE)
            else:
                stdscr.addstr(9 + i, 0, f"Element {i}: {val}")
        stdscr.refresh()

    def control_loop(self, stdscr):
        curses.curs_set(0)  # Hide the cursor
        
        while True:
            self.display(stdscr)
            key = stdscr.getch()

            if self.mode == utils.PanelState.NORMAL:
                if key == ord('q'):
                    break  # Exit the control loop and script
                elif key == curses.KEY_DOWN:
                    self.index = (self.index + 1) % len(self.array)
                elif key == curses.KEY_UP:
                    self.index = (self.index - 1) % len(self.array)
                elif key == curses.KEY_LEFT or key == curses.KEY_RIGHT:
                    self.switch_array()  # Switch between left and right arrays
                elif key == ord('i'):
                    self.mode = utils.PanelState.CONTROL

            elif self.mode == utils.PanelState.CONTROL:
                if key == curses.KEY_UP:
                    self.array[self.index] += 1  # Increase current array value
                elif key == curses.KEY_DOWN:
                    self.array[self.index] -= 1  # Decrease current array value
                elif key == 27:  # ESC key
                    self.mode = utils.PanelState.NORMAL


def main(stdscr):
    panel = ControlPanel()
    panel.control_loop(stdscr)


if __name__ == "__main__":
    curses.wrapper(main)
