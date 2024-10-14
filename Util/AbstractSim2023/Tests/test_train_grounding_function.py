"""
import numpy as np
import sys
sys.path.append(sys.path[0] + "/..")

from utils.train_grounding_function import get_grounding_function_training_set, collision_check
"""

import pytest

@pytest.mark.skip(reason="Deprecated")
def test_get_grounding_function_training_set():
    trajectories = {
        "abstract_states": [
            [1 for i in range(10)],
            [1.1 for i in range(10)],
            [1.2 for i in range(10)],
        ],
        "actions": [[1, 1, 1], [0.9, 0.9, 0.9], [0.8, 0.8, 0.8]],
        "episode_starts": [0, 0, 0],
    }

    training_set = get_grounding_function_training_set(trajectories, {"history_length":2, "environment":"dummy_defenders", "num_original_data": 1, "num_augmented_data": 0})

    assert len(training_set["X"]) == 1
    assert len(training_set["y"]) == 1

    training_set = get_grounding_function_training_set(trajectories, {"history_length":1, "environment":"dummy_defenders", "num_original_data": 1, "num_augmented_data": 0})

    assert len(training_set["X"]) == 2
    assert len(training_set["y"]) == 2

    training_set = get_grounding_function_training_set(trajectories, {"history_length":1, "environment":"dummy_defenders", "num_original_data": 1, "num_augmented_data": 1})

    assert len(training_set["X"]) == 4
    assert len(training_set["y"]) == 4

@pytest.mark.skip(reason="Deprecated")
def test_collision_check():


    object_1_x = np.array([0,0,0])
    object_1_y = np.array([1,1,1])

    object_2_x = np.array([1,1,0])
    object_2_y = np.array([1,0,2])

    assert collision_check(object_1_x, object_1_y, object_2_x, object_2_y, .001) == False
    assert collision_check(object_1_x, object_1_y, object_2_x, object_2_y, 1.0) == False
    assert collision_check(object_1_x, object_1_y, object_2_x, object_2_y, 1.001) == True







