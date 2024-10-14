from envs.push_ball_to_goal import PushBallToGoalEnv
import sys
sys.path.append(sys.path[0] + "/..")


def test_get_abstract_state():

    env = PushBallToGoalEnv()

    env.robot_x = 335
    env.robot_y = 234

    env.target_x = 435
    env.target_y = 123

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
    print(env.get_abstract_state())

    assert abs(env.target_x - 435.0) <= 0.000001
    assert env.target_y == 123.0
    assert abs(env.robot_x - 335.0) <= 0.000001
    assert env.robot_y == 234.0
    assert abs(env.robot_angle - 1.4) <= 0.000001

    abstract_state = [
        0.510203,
        0.099905,
        0.510203,
        0.099905,
        0.0155505,
        -0.0583747,
        -0.00534717,
        0.15828,
        0.999331,
        0.0365775,
    ]
    env.reset()
    env.set_abstract_state(abstract_state)
    new_state = env.get_abstract_state().tolist()
    for entry_1, entry_2 in zip(new_state, abstract_state):
        assert (abs(entry_1 - entry_2)) <= 0.00001
