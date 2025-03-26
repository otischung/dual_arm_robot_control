import pytest
from unittest.mock import MagicMock
from dual_arm_robot_control.control_panel import TUI, NormalState, SelectJointState, ControlJointState, SelectParamState, ControlParamState, ControlParamModeState, ExitState
from dual_arm_robot_control.arm_publisher import ArmPublisher
from dual_arm_robot_control.params import *
from dual_arm_robot_control.utils import *


@pytest.fixture
def mock_arm_publisher():
    return MagicMock(spec=ArmPublisher)


@pytest.fixture
def tui(mock_arm_publisher):
    return TUI(mock_arm_publisher)


def test_initial_state(tui):
    assert isinstance(tui.state, NormalState)
    assert tui.cur_sel == PanelSelect.LEFT
    assert tui.cur_sel_param == ParamSelect.STEP
    assert tui.cur_sel_joint == 0
    assert tui.cur_joint_left == DEFAULT_LEFT_JOINT_DEG_ANGLE
    assert tui.cur_joint_right == DEFAULT_RIGHT_JOINT_DEG_ANGLE
    assert tui.step == DEFAULT_JOINT_MOVE_STEP_DEG
    assert tui.param_mode == ParamMode.SPEED
    assert tui.param == DEFAULT_SPEED_DEG_PER_SEC
    assert tui.fps == DEFAULT_FPS


def test_select_next_item(tui):
    tui.select_next_item()
    assert tui.cur_sel == PanelSelect.RIGHT


def test_select_prev_item(tui):
    tui.select_prev_item()
    assert tui.cur_sel == PanelSelect.RESET_PARAM


def test_change_state(tui):
    tui.change_state(SelectJointState)
    assert isinstance(tui.state, SelectJointState)


def test_set_to_left(tui):
    tui.set_to_left()
    assert tui.cur_sel_arm == tui.cur_joint_left
    assert tui.cur_sel_arm_min == MIN_LEFT_JOINT_DEG_ANGLE
    assert tui.cur_sel_arm_max == MAX_LEFT_JOINT_DEG_ANGLE


def test_set_to_right(tui):
    tui.set_to_right()
    assert tui.cur_sel_arm == tui.cur_joint_right
    assert tui.cur_sel_arm_min == MIN_RIGHT_JOINT_DEG_ANGLE
    assert tui.cur_sel_arm_max == MAX_RIGHT_JOINT_DEG_ANGLE


def test_select_prev_joint(tui):
    tui.cur_sel_joint = 1
    tui.select_prev_joint()
    assert tui.cur_sel_joint == 0


def test_select_next_joint(tui):
    tui.cur_sel_joint = 0
    tui.select_next_joint()
    assert tui.cur_sel_joint == 1


def test_increase_joint(tui):
    tui.cur_sel_joint = 0
    tui.increase_joint()
    assert tui.cur_sel_arm[tui.cur_sel_joint] == DEFAULT_LEFT_JOINT_DEG_ANGLE[0] + tui.step


def test_decrease_joint(tui):
    tui.cur_sel_joint = 0
    tui.decrease_joint()
    assert tui.cur_sel_arm[tui.cur_sel_joint] == DEFAULT_LEFT_JOINT_DEG_ANGLE[0] - tui.step


def test_select_prev_param(tui):
    tui.cur_sel_param = ParamSelect.PARAM
    tui.select_prev_param()
    assert tui.cur_sel_param == ParamSelect.MODE


def test_select_next_param(tui):
    tui.cur_sel_param = ParamSelect.STEP
    tui.select_next_param()
    assert tui.cur_sel_param == ParamSelect.MODE


def test_select_prev_mode(tui):
    tui.param_mode = ParamMode.SPEED
    tui.select_prev_mode()
    assert tui.param_mode == ParamMode.DURATION


def test_select_next_mode(tui):
    tui.param_mode = ParamMode.SPEED
    tui.select_next_mode()
    assert tui.param_mode == ParamMode.DURATION


def test_reset_angle(tui):
    tui.reset_angle()
    assert tui.cur_sel == PanelSelect.LEFT
    assert tui.cur_joint_left == DEFAULT_LEFT_JOINT_DEG_ANGLE
    assert tui.cur_joint_right == DEFAULT_RIGHT_JOINT_DEG_ANGLE


def test_reset_param(tui):
    tui.reset_param()
    assert tui.cur_sel == PanelSelect.LEFT
    assert tui.step == DEFAULT_JOINT_MOVE_STEP_DEG
    assert tui.param_mode == ParamMode.SPEED
    assert tui.param == DEFAULT_SPEED_DEG_PER_SEC
    assert tui.fps == DEFAULT_FPS
