from utils.score_rollouts import split_episodes
import sys
sys.path.append(sys.path[0] + "/..")


def test_split_episodes():

    trajectories = {
        "episode_starts": [True, False, False, True, False],
        "abstract_states": [1, 2, 3, 4, 5],
        "actions": [6, 7, 8, 9, 10],
    }

    episodes = split_episodes(trajectories)

    assert episodes == [
        {"abstract_states": [1, 2, 3], "actions": [6, 7, 8]},
        {"abstract_states": [4, 5], "actions": [9, 10]},
    ]

    trajectories = {
        "episode_starts": [True, False, False, True, True],
        "abstract_states": [1, 2, 3, 4, 5],
        "actions": [6, 7, 8, 9, 10],
    }

    episodes = split_episodes(trajectories)

    assert episodes == [
        {"abstract_states": [1, 2, 3], "actions": [6, 7, 8]},
        {"abstract_states": [4], "actions": [9]},
        {"abstract_states": [5], "actions": [10]},
    ]
