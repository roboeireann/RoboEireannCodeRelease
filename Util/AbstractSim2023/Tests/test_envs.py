import pytest
import sys
sys.path.append(sys.path[0] + "/..")


from envs.walk_to_goal import WalkToGoalEnv
from envs.walk_to_ball import WalkToBallEnv
from envs.push_ball_to_goal import PushBallToGoalEnv
from envs.dummy_defenders import DummyDefendersEnv
from envs.goalie import GoalieEnv


right_goal_envs = [
    WalkToBallEnv,
    PushBallToGoalEnv,
]
left_goal_envs = [
    DummyDefendersEnv,
    GoalieEnv,
]



# tests to make sure all environments conform to standard values for certain
# constants.
@pytest.mark.parametrize("env_class", right_goal_envs)
def test_standard_values(env_class):

    env = env_class()
    env.reset()

    #assert env.goal_x == 4800 temporarily disabled because environments have different goal positions
    assert env.goal_y == 0

    assert env.displacement_coef == 0.2


@pytest.mark.parametrize("env_class", right_goal_envs)
def test_get_set_abstract_state(env_class):

    env = env_class()
    env.reset()

    env.robot_x = 335
    env.robot_y = 234

    env.target_x = 435
    env.target_y = 123

    env.dummy1_x = 546
    env.dummy1_y = 204

    env.dummy2_x = 991
    env.dummy2_y = 769

    env.robot_angle = 1.4

    # print(env.robot_angle)

    state = env.get_abstract_state()
    print(state)

    # print(env.robot_angle)

    env.robot_x = 0
    env.robot_y = 0

    env.target_x = 0
    env.target_y = 0

    env.robot_angle = 0

    # print(state)
    env.reset()

    env.set_abstract_state(state)

    assert (env.target_x - 435.0) <= 0.000001
    assert (env.target_y - 123.0) <= 0.000001
    assert (env.robot_x - 335.0) <= 0.000001
    assert (env.robot_y - 234.0) <= 0.000001
    assert (env.dummy1_x - 546.0) <= 0.000001
    assert (env.dummy1_y - 204.0) <= 0.000001
    assert (env.dummy2_x - 991.0) <= 0.000001
    assert (env.dummy2_y - 769.0) <= 0.000001
    assert abs(env.robot_angle) - 1.4 <= 0.000001
