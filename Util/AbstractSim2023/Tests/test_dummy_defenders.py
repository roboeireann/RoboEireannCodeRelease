import sys
sys.path.append(sys.path[0] + "/..")
from envs.dummy_defenders import DummyDefendersEnv



"""
def test_get_set_abstract_state():

    env = DummyDefendersEnv()

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

    assert env.target_x == 435.0
    assert env.target_y == 123.0
    assert env.robot_x == 335.0
    assert env.robot_y == 234.0
    assert env.dummy1_x == 546.0
    assert env.dummy1_y == 204.0
    assert env.dummy2_x == 991.0
    assert env.dummy2_y == 769.0
    assert abs(env.robot_angle) - 1.4 <= 0.000001
"""
