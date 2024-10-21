from enum import Enum


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


# Declare the enum for LEFT and RIGHT arm
class ArmSide(Enum):
    LEFT = 1
    RIGHT = 2


# Declare the enum for state of control panel
class PanelState(Enum):
    NORMAL = 1
    CONTROL = 2
