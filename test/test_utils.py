import pytest
import curses

from dual_arm_robot_control.utils import (
    count_trailing_zeros_bitwise,
    get_len_bitwise_enum,
    get_max_bitwise_enum,
    next_bitwise_enum,
    prev_bitwise_enum,
    key_trans,
    ArmSide,
    PanelSelect,
    PanelState,
    ParamMode,
    ParamSelect
)
from dual_arm_robot_control.params import *


def test_count_trailing_zeros_bitwise():
    assert count_trailing_zeros_bitwise(0x00000000) == 0
    assert count_trailing_zeros_bitwise(0x00000001) == 0
    assert count_trailing_zeros_bitwise(0x00000002) == 1
    assert count_trailing_zeros_bitwise(0x00000004) == 2
    assert count_trailing_zeros_bitwise(0x00000008) == 3
    assert count_trailing_zeros_bitwise(0x00000010) == 4


def test_get_len_bitwise_enum():
    assert get_len_bitwise_enum(ArmSide) == 2
    assert get_len_bitwise_enum(PanelSelect) == 5
    assert get_len_bitwise_enum(PanelState) == 6


def test_get_max_bitwise_enum():
    assert get_max_bitwise_enum(ArmSide) == 2
    assert get_max_bitwise_enum(PanelSelect) == 16
    assert get_max_bitwise_enum(PanelState) == 32


def test_next_bitwise_enum():
    assert next_bitwise_enum(ArmSide.LEFT, ArmSide) == ArmSide.RIGHT
    assert next_bitwise_enum(ArmSide.RIGHT, ArmSide) == ArmSide.LEFT
    assert next_bitwise_enum(
        PanelSelect.LEFT, PanelSelect) == PanelSelect.RIGHT
    assert next_bitwise_enum(PanelSelect.RESET_PARAM,
                             PanelSelect) == PanelSelect.LEFT


def test_prev_bitwise_enum():
    assert prev_bitwise_enum(ArmSide.LEFT, ArmSide) == ArmSide.RIGHT
    assert prev_bitwise_enum(ArmSide.RIGHT, ArmSide) == ArmSide.LEFT
    assert prev_bitwise_enum(
        PanelSelect.LEFT, PanelSelect) == PanelSelect.RESET_PARAM
    assert prev_bitwise_enum(PanelSelect.RESET_PARAM,
                             PanelSelect) == PanelSelect.RESET_ANGLE


def test_key_trans():
    assert key_trans(curses.KEY_ENTER) == KEY_ENTER
    assert key_trans(10) == KEY_ENTER
    assert key_trans(13) == KEY_ENTER
    assert key_trans(ord('q')) == KEY_ESC
    assert key_trans(curses.KEY_UP) == curses.KEY_UP
    assert key_trans(curses.KEY_DOWN) == curses.KEY_DOWN
