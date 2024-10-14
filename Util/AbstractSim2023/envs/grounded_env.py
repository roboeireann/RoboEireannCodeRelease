import copy
import gym
import torch
import numpy as np


class HistoryBuffer:
    def __init__(self, history_length):
        self.action_buffer = []
        self.state_buffer = []
        self.history_length = history_length

    def push(self, state, action):
        assert isinstance(state, list)
        assert isinstance(action, list)

        assert len(state) == 10
        assert len(action) == 3

        # For the first frame of the episode we act as if the last n frames were identical to the current frame, where n is the length of the history buffer.
        if len(self.state_buffer) == 0:
            self.state_buffer = [
                copy.deepcopy(state) for i in range(self.history_length)
            ]
            self.action_buffer = [
                copy.deepcopy(action) for i in range(self.history_length)
            ]
        else:
            self.state_buffer = self.state_buffer[1:]
            self.state_buffer.append(copy.deepcopy(state))
            self.action_buffer = self.action_buffer[1:]
            self.action_buffer.append(copy.deepcopy(action))

        assert len(self.state_buffer) == self.history_length
        assert len(self.action_buffer) == self.history_length

        for entry in self.state_buffer:
            assert len(entry) == 10
        for entry in self.action_buffer:
            assert len(entry) == 3

    def get_grounding_input(self, state_estimate):

        state_estimate = list(state_estimate)

        result = (
            [item for entry in self.state_buffer for item in entry]
            + state_estimate
            + [item for entry in self.action_buffer for item in entry]
        )

        assert len(result) == ((self.history_length + 1) * 10) + (
            self.history_length * 3
        )
        return result

    def clear(self):
        self.buffer = []


class GroundedEnv(gym.Env):
    def __init__(
        self,
        env,
        grounding_net,
        standardization_info,
        history_length,
        stochastic=False,
        grounding_strength=1,
    ):
        self.env = env
        self.history_length = history_length
        self.stochastic = stochastic
        self.X_mean = standardization_info["X_mean"]
        self.y_mean = standardization_info["y_mean"]
        self.X_std = standardization_info["X_std"]
        self.y_std = standardization_info["y_std"]
        self.avg_error = standardization_info["avg_error"]
        assert grounding_strength == 1, "grounding strength should always be 1"
        self.grounding_strength = grounding_strength

        self.grounding_net = grounding_net
        self.grounding_net.eval()
        self.observation_space = self.env.observation_space
        self.action_space = self.env.action_space
        self.reset()

    def reset(self, initial_state = None):
        if initial_state == None: # we allow an initial state to be specified when resetting
            self.prev_obs = self.env.reset()
        else:
            self.env.set_abstract_state(initial_state)
            self.prev_obs = self.env._observe_state()
        self.prev_state = self.env.get_abstract_state()
        self.history_buffer = HistoryBuffer(self.history_length)
        return self.prev_obs

    def step(self, action):


        action = list(action)

        prev_state = self.env.get_abstract_state()

        obs, reward, done, info = self.env.step(action)

        self.history_buffer.push(list(prev_state), action)

        state_estimate = self.env.get_abstract_state()

        grounding_input = torch.Tensor(
            self.history_buffer.get_grounding_input(state_estimate)
        )

        offset = None
        if self.stochastic == False:

            # print(self.X_std)
            # print(self.X_mean)

            # print(grounding_input)
            grounding_input = (
                grounding_input - torch.Tensor(self.X_mean)
            ) / torch.Tensor(self.X_std)

            # print(grounding_input)

            offset = self.grounding_net(grounding_input).detach().numpy()

            # offset = np.random.multivariate_normal(offset,np.diag(self.avg_error ** 2))

            offset = (torch.Tensor(offset) * torch.Tensor(self.y_std)) + self.y_mean

            offset = offset.numpy()

            # print(offset)
            # ffset = offset + torch.Tensor(self.y_mean).numpy()
            # print(offset)
            # rint("end")
        else:
            # outdated
            offset = (
                self.grounding_net(grounding_input.reshape((1, -1)))
                .detach()
                .numpy()
                .reshape((10,))
            )

        # print("offset")
        # print(offset)

        new_state = tuple(
            [
                float(state_entry + self.grounding_strength * offset_entry)
                for state_entry, offset_entry in zip(state_estimate, offset)
            ]
        )

        """
        new_state = []
        for i in range(0,len(temp_new_state)):
            if i > 8:
                new_state.append(state_estimate[i])
            else:
                new_state.append(temp_new_state[i])
        new_state = tuple(new_state)
        """
        # new_state = state_estimate

        # self.env.render()
        # input("press enter")
        self.env.set_abstract_state(new_state)
        # self.env.render()

        return (self.env._observe_state(), self.env.calculate_reward(), done, info)

    def render(self):
        self.env.render()
