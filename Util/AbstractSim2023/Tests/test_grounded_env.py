"""
import numpy as np
import sys
sys.path.append(sys.path[0] + "/..")

from envs.grounded_env import HistoryBuffer, GroundedEnv
from envs.push_ball_to_goal import PushBallToGoalEnv
from utils.train_grounding_function import Network
"""
import pytest

@pytest.mark.skip(reason="Deprecated")
def test_history_buffer():
    buffer = HistoryBuffer(2)

    state = list((1, 1, 1, 1, 1, 1, 1, 1, 1, 1))
    action = list((3, 3, 3))

    buffer.push(state, action)

    state_estimate = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2]

    grounding_input = tuple(buffer.get_grounding_input(state_estimate))

    assert grounding_input == ((1,) * 10 * 2) + ((2,) * 10) + ((3,) * 3 * 2)

    new_state = [4, 4, 4, 4, 4, 4, 4, 4, 4, 4]
    new_action = [5, 5, 5]
    buffer.push(new_state, new_action)
    grounding_input = tuple(buffer.get_grounding_input(state_estimate))

    assert grounding_input == ((1,) * 10 * 1) + ((4,) * 10 * 1) + ((2,) * 10) + (
        (3,) * 3 * 1
    ) + ((5,) * 3 * 1)

@pytest.mark.skip(reason="Deprecated")
@pytest.mark.parametrize("history_length", [1, 2, 3, 4, 5])
def test_grounded_env(history_length):

    net = Network(history_length)

    standardization_info = {
        "X_mean": np.zeros(10 * (history_length + 1) + (3 * history_length)),
        "y_mean": np.zeros(10),
        "X_std": np.zeros(10 * (history_length + 1) + (3 * history_length)),
        "y_std": np.ones(10),
        "avg_error": np.ones(10) * 0.0001,
    }

    env = PushBallToGoalEnv()
    env = GroundedEnv(env, net, standardization_info, history_length)

    env.reset()

    assert len(env.history_buffer.action_buffer) == 0
    assert len(env.history_buffer.state_buffer) == 0

    _, _, _, _ = env.step([1, 1, 1])

    assert len(env.history_buffer.action_buffer) == history_length
    assert len(env.history_buffer.state_buffer) == history_length

    _, _, _, _ = env.step([1, 1, 1])

    assert len(env.history_buffer.action_buffer) == history_length
    assert len(env.history_buffer.state_buffer) == history_length
