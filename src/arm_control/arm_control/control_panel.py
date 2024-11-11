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
            {"src": PanelState.NORMAL, "dst": PanelState.NORMAL,
                "key": KEY_ENTER, "sel": PanelSelect.RESET},
            {"src": PanelState.NORMAL, "dst": PanelState.SELECT,
                "key": KEY_ENTER, "sel": PanelSelect.LEFT | PanelSelect.RIGHT},
            {"src": PanelState.NORMAL, "dst": PanelState.CONTROL_STEP,
                "key": KEY_ENTER, "sel": PanelSelect.STEP},
            {"src": PanelState.NORMAL, "dst": PanelState.EXIT,
                "key": KEY_ESC, "sel": PanelSelect.ALL},
            {"src": PanelState.SELECT, "dst": PanelState.NORMAL,
                "key": KEY_ESC, "sel": PanelSelect.LEFT | PanelSelect.RIGHT},
            {"src": PanelState.SELECT, "dst": PanelState.CONTROL_JOINT,
                "key": KEY_ENTER, "sel": PanelSelect.LEFT | PanelSelect.RIGHT},
            {"src": PanelState.CONTROL_JOINT, "dst": PanelState.SELECT,
                "key": KEY_ESC, "sel": PanelSelect.LEFT | PanelSelect.RIGHT},
            {"src": PanelState.CONTROL_STEP, "dst": PanelState.NORMAL,
                "key": KEY_ESC, "sel": PanelSelect.STEP},
        ]
        self._cur_state: PanelState = PanelState.NORMAL
        self._cur_sel: PanelSelect = PanelSelect.LEFT
        self._cur_sel_joint: int = 0

    def control_loop(self, stdscr):
        while True:
            stdscr.clear()
            stdscr.addstr(1, 0, f"{self._cur_state.name}")
            key = key_trans(stdscr.getch())
            trans: dict = None

            for trans_ in self._transitions:
                # Check if match.
                if trans_["key"] == key and \
                        bool(trans_["src"] & self._cur_state) and \
                        bool(trans_["sel"] & self._cur_sel):
                    trans = trans_
                    break

            if trans is None:
                stdscr.addstr(2, 0, "Error, there is NO matched transition.")
            else:
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
