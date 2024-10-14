# derived from https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html

import gym
import torch
import numpy as np
import json
import torch
import torch.nn as nn
import torch.nn.functional as functional
from torch.utils.data import TensorDataset, DataLoader
import torch.optim as optim
from tensorflow import keras
import tensorflow as tf
from torch.distributions.multivariate_normal import MultivariateNormal
import math
from threading import Thread
import os.path
import time
import sys
sys.path.append(sys.path[0] + "/..")
import pickle
import random

from networks import export_network

from utils.utils import get_json_from_file_name

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

from tqdm import tqdm
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecMonitor, VecNormalize

from envs.push_ball_to_goal import PushBallToGoalEnv
from envs.dummy_defenders import DummyDefendersEnv
from envs.goalie import GoalieEnv
from utils.score_rollouts import split_episodes

from utils.merge_trajectories import merge_trajectories

from utils.utils import save_vec_normalize_data
from utils.grounded_env import GroundedEnv

#BATCH_SIZE = 20000
#TRAIN_EPOCHS = 400  # 100 for non-physical experiments, 400 for physical experiments
# TRAINING_ENTRIES = 100000
VAL_ENTRIES = 3000

GROUNDING_STRENGTH = 1  # .5 for non-physical experiments, .05 for physical experiments

#ENVIRONMENT = DummyDefendersEnv

ENVIRONMENTS = {'push_ball_to_goal': PushBallToGoalEnv,'dummy_defenders': DummyDefendersEnv, 'goalie': GoalieEnv}

NETWORK = []

#HISTORY_LENGTH = 20  # 20 for non-physical experiments, 3 for physical experiments


def plot_losses(losses, val_losses, zero_losses):
    plt.plot(losses, color="Blue")
    plt.plot(val_losses, color="Orange")
    plt.plot(zero_losses)
    plt.show()


class Network(nn.Module):
    def __init__(self, history_length):
        super().__init__()
        self.dense1 = nn.Linear(10 * (history_length + 1) + 3 * history_length, 100)
        self.dense2 = nn.Linear(100, 100)
        self.dense3 = nn.Linear(100, 50)
        self.dense4 = nn.Linear(50, 10)

    def forward(self, x):
        x = functional.relu(self.dense1(x))
        x = functional.relu(self.dense2(x))
        x = functional.relu(self.dense3(x))
        x = self.dense4(x)
        return x



def collision_check(object_1_x_array, object_1_y_array, object_2_x_array, object_2_y_array, radius):
    distances = np.sqrt((object_1_x_array - object_2_x_array) ** 2 + (object_1_y_array - object_2_y_array) ** 2)
    #print(distances)
    if np.min(distances) < radius:
        return True
    else:
        return False
    


def positional_invariance_randomization(abstract_states, history_length):
    """Here we provide a randomization to shift around the positions of objects in training samples where they
    aren't near each other. We don't want to grounding_function to overfit behaviors to world positions"""
    assert len(abstract_states) == (history_length + 1) 


    robot_x_array = np.array([abstract_state[0] for abstract_state in abstract_states])
    robot_y_array = np.array([abstract_state[1] for abstract_state in abstract_states])

    ball_x_array = np.array([abstract_state[2] for abstract_state in abstract_states])
    ball_y_array = np.array([abstract_state[3] for abstract_state in abstract_states])


    min_robot_x = np.min(robot_x_array)
    max_robot_x = np.max(robot_x_array)

    min_robot_y = np.min(robot_y_array)
    max_robot_y = np.max(robot_y_array)

    min_ball_x = np.min(ball_x_array)
    max_ball_x = np.max(ball_x_array)

    min_ball_y = np.min(ball_y_array)
    max_ball_y = np.max(ball_y_array)


    x_shift = random.uniform(-max(min_robot_x + 1,0),max(0,1-max_robot_x))
    y_shift = random.uniform(-max(min_robot_y + 1,0),max(0,1-max_robot_y))


    for abstract_state in abstract_states:
        abstract_state[0] += x_shift
        abstract_state[1] += y_shift

    
    while collision_check(ball_x_array, ball_y_array,robot_x_array, robot_y_array, radius =.01):
        x_shift = random.uniform(-max(min_ball_x + 1,0),max(0,1-max_ball_x))
        y_shift = random.uniform(-max(min_ball_y + 1,0),max(0,1-max_ball_y))
        ball_x_array += x_shift
        ball_y_array += y_shift
        for abstract_state in abstract_states:
            abstract_state[2] += x_shift
            abstract_state[3] += y_shift

    return abstract_states

def get_grounding_function_training_set(trajectories, rl_config):


    HISTORY_LENGTH = rl_config['history_length']


    training_set = {"X": [], "y": []}

    ENVIRONMENT = ENVIRONMENTS[rl_config['environment']]

    env = ENVIRONMENT()

    observations = trajectories["abstract_states"]
    actions = trajectories["actions"]
    episode_starts = trajectories["episode_starts"]

    num_original_data = rl_config['num_original_data']
    num_augmented_data = rl_config['num_augmented_data']

    # print(observations)
    # print(actions)
    # print(episode_starts)
    HISTORY_BUFFER_LENGTH = HISTORY_LENGTH + 1
    # HISTORY_BUFFER_LENGTH = temp + 1
    print(HISTORY_BUFFER_LENGTH)
    print(len(observations))
    print(len(actions))



    for current_observation_set_num in range(num_original_data + num_augmented_data):

        for i in range(len(observations) - HISTORY_LENGTH):

            # i = random.randrange(0,len(observations) - history_length )
            print(i)
            sample_observations = observations[i : i + HISTORY_BUFFER_LENGTH]
            sample_actions = actions[i : i + HISTORY_BUFFER_LENGTH - 1]
            sample_starts = episode_starts[i : i + HISTORY_BUFFER_LENGTH]
            skip_flag = False
            for c in range(len(sample_starts)):
                if sample_starts[c] == 1 and c != 0:
                    skip_flag = True
            if skip_flag:
                continue

            # print("sample_observations")
            # print(sample_observations)

            abstract_state = sample_observations[-2]
            # print("abstract_state")
            # print(abstract_state)
            env.set_abstract_state(abstract_state)
            # print("after set")
            # print(env.get_abstract_state())
            env.step(sample_actions[-1])
            true_final_state = sample_observations[-1]
            abstract_sim_final_state = env.get_abstract_state().tolist()
            assert float("NaN") not in abstract_sim_final_state
            sample_observations[-1] = abstract_sim_final_state

            # print("true final state")
            # print(true_final_state)
            # print("estimated final state")
            # print(sample_observations[-1])
            # print(sample_actions)
            # print("diff")
            # print(true_final_state)
            # print(abstract_sim_final_state)
            # exit()
            assert len(true_final_state) == 10
            assert len(abstract_sim_final_state) == 10
            y = tuple(
                [
                    true_entry - abstract_entry
                    for true_entry, abstract_entry in zip(
                        true_final_state, abstract_sim_final_state
                    )
                ]
            )
            for element in y:
                assert not math.isnan(element)
                assert isinstance(element, float)
                assert np.isfinite(element)
            # if max(y) > .1 or min(y) < -.1:
            #    continue

            if num_original_data - current_observation_set_num <= num_augmented_data:
                sample_observations = positional_invariance_randomization(sample_observations, rl_config['history_length'])


            # print(y)
            training_set["X"].append(
                [
                    float(i)
                    for table in [sample_observations, sample_actions]
                    for entry in table
                    for i in entry
                ]
            )

            # print(len(sample_observations))
            # print(len(sample_actions))
            # print("y")
            # print(y)
            training_set["y"].append(y)

            # print(training_set["X"])
            # print(training_set["y"])
            # exit()
        #BATCH_SIZE = len(training_set["X"])
        # print(training_set['X'].shape)
        # print(training_set['y'].shape)

    return training_set


def visualization_phase(rl_config):



    HISTORY_LENGTH = rl_config['history_length']
    GROUNDING_STRENGTH = 1

    if rl_config['environment'] not in ENVIRONMENTS:
        print("invalid environment")
        exit(1)

    ENVIRONMENT = ENVIRONMENTS[rl_config['environment']]
    IS_CONTROL = rl_config['control']

    env_singleton = ENVIRONMENT


    env_lambda = None
    if not IS_CONTROL:
        standardization_info = None
        with open(f"{rl_config['output_path']}/standardization_info.pkl", 'rb') as standardization_info_file:
            standardization_info = pickle.load(standardization_info_file)
        net = NETWORK(HISTORY_LENGTH)

        net.load_state_dict(torch.load(f"{rl_config['output_path']}/sim_grounding_function"))
        net.training = False
        env_lambda = lambda: GroundedEnv(
            env_singleton(),
            net,
            standardization_info,
            HISTORY_LENGTH,
            stochastic=False,
            grounding_strength=GROUNDING_STRENGTH,
        )
    else:
        env_lambda = env_singleton


    env = VecNormalize.load(
        f"{rl_config['output_path']}/retrained_policy_normalize",
        make_vec_env(env_lambda, n_envs=1),
    )
    env.clip_obs = 1.0
    env.norm_obs = True
    env.norm_reward = True
    env.training = True

    model = PPO.load( f"{rl_config['output_path']}/retrained_policy", env)

    obs = env.reset()
    while True:
        time.sleep(0.08)
        action = model.predict(obs)[0].tolist()
        print(action[0])
        # print(obs)
        obs, reward, done, _ = env.step(action)
        env.envs[0].env.render()
        if done[0]:
            obs = env.reset()



def grounding_phase(rl_config):

    HISTORY_LENGTH = rl_config['history_length']


    #if not exists [rlconfig['output_path]/merged_trajectories TODO 
    merge_trajectories(rl_config['trajectories_source'], f"{rl_config['output_path']}/merged_trajectories.json", rl_config['train_val_split'])

    trajectories = get_json_from_file_name(f"{rl_config['output_path']}/merged_trajectories.json")
    samples = get_grounding_function_training_set(trajectories, rl_config)
    val_trajectories = get_json_from_file_name(f"{rl_config['output_path']}/merged_trajectories_val.json")
    val_samples = get_grounding_function_training_set(val_trajectories,rl_config)
    

    #TODO think about how to handle checkpointing here 
    with open(f"{rl_config['output_path']}/samples_file.json", 'w') as samples_file:
        json.dump(samples, samples_file)

    with open(f"{rl_config['output_path']}/val_samples_file.json", 'w') as val_samples_file:
        json.dump(val_samples, val_samples_file)

    
    net, losses, val_losses,zero_losses, X_mean, y_mean, X_std, y_std, avg_error = train_grounding_function(samples,val_samples, rl_config)


    standardization_info = {'X_mean':X_mean, 'y_mean':y_mean, 'X_std':X_std, 'y_std':y_std, 'avg_error': avg_error}

    print(standardization_info)
    

    with open(f"{rl_config['output_path']}/standardization_info.pkl", "wb") as info_file:
        pickle.dump(standardization_info, info_file, protocol = 4)



    np.save(f"{rl_config['output_path']}/standardization_info", standardization_info)


    torch.save(net.state_dict(), f"{rl_config['output_path']}/sim_grounding_function")


    
    print(losses)
    print(val_losses)
    print(zero_losses)
    plot_losses(losses, val_losses, zero_losses)
    

def grounded_retraining_phase(rl_config):
    #standardization_info = np.load("./standardization_info.npy", allow_pickle=True)
    
    IS_CONTROL = rl_config['control']

    
    standardization_info = None
    if not IS_CONTROL:
        with open(f"{rl_config['output_path']}/standardization_info.pkl", 'rb') as standardization_info_file:
            standardization_info = pickle.load(standardization_info_file)
    
    HISTORY_LENGTH = rl_config['history_length']

    RL_TRAINING_TIMESTEPS = rl_config['rl_train_timesteps']

    GROUNDING_STRENGTH = 1


    

    if rl_config['environment'] not in ENVIRONMENTS:
        print("invalid environment")
        exit(1)

    ENVIRONMENT = ENVIRONMENTS[rl_config['environment']]

    env_singleton = ENVIRONMENT

    
    #env_lambda = lambda : env_singleton()

    net = None
    if not IS_CONTROL:
        net = NETWORK(HISTORY_LENGTH)
        net.load_state_dict(torch.load(f"{rl_config['output_path']}/sim_grounding_function"))
        net.training = False

    env_lambda = None
    if not IS_CONTROL:
        env_lambda = lambda: GroundedEnv(
            env_singleton(),
            net,
            standardization_info,
            HISTORY_LENGTH,
            stochastic=False,
            grounding_strength=GROUNDING_STRENGTH,
        )
    else:
        env_lambda = env_singleton

    #training_trajectory_comparison(net, val_trajectories, standardization_info)
    #exit()

    
    env = VecNormalize.load(f"{rl_config['starter_policy']}/vector_normalize",make_vec_env(env_lambda, n_envs = 12))
    env.clip_obs = 1.0
    env.norm_obs = True
    env.norm_reward = True
    env.training = True
    
    save_vec_normalize_data(env, f"{rl_config['output_path']}/before_retraining_normalize_metadata.json")


    env = VecMonitor(venv=env, filename =  "result_")


    model = PPO.load(f"{rl_config['starter_policy']}/policy", env)

    
    model.learn(total_timesteps = RL_TRAINING_TIMESTEPS) 

    model.save(f"{rl_config['output_path']}/retrained_policy")
    env.save(f"{rl_config['output_path']}/retrained_policy_normalize")
    save_vec_normalize_data(env, f"{rl_config['output_path']}/retrained_normalize_metadata.json")

    



def train_grounding_function(experience_samples, validation_samples, rl_config):

    HISTORY_LENGTH = rl_config['history_length']
    BATCH_SIZE = rl_config['batch_size']
    TRAIN_EPOCHS = rl_config['train_epochs']

    stochastic = False #TODO remove this deprecated

    data_dict = experience_samples

    X = torch.Tensor(data_dict["X"])
    y = torch.Tensor(data_dict["y"])

    print(X.shape)
    print(y.shape)

    X_mean = X.mean(axis=0).detach()
    y_mean = y.mean(axis=0).detach()
    X_std = X.std(axis=0).detach()
    y_std = y.std(axis=0).detach()

    X_std = X_std.apply_(lambda x : x if x != 0 else .001) #TODO this shouldnt be hardcoded
    y_std = y_std.apply_(lambda x : x if x != 0 else .001)




    print(X_mean.shape)
    print(y_mean.shape)

    X = (X - X_mean) / X_std
    y = (y - y_mean) / y_std

    dataset = TensorDataset(X, y)
    # val_dataset = TensorDataset(torch.Tensor(validation_samples['X']),torch.Tensor(validation_samples['y']))
    experience_samples = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True)

    net = None
    if stochastic:
        net = StochasticNetwork(HISTORY_LENGTH)
    else:
        net = NETWORK(HISTORY_LENGTH)


    losses = []
    val_losses = []
    zero_losses = []

    optimizer = optim.Adam(net.parameters(), lr=0.001)
    trange = tqdm(range(TRAIN_EPOCHS))
    for i in trange:
        for batch in experience_samples:
            # print(batch)
            batch_X, batch_y = batch
            net.zero_grad()

            
            print("x")
            print(batch_X)
            print("y")
            print(batch_y)

            means = torch.mean(batch_y, 0)
            stds = torch.std(batch_y, 0)

            loss = None
            if stochastic:
                mean, covariance_diagonal = net(batch_X)
                loss = -MultivariateNormal(
                    mean, torch.diag_embed(covariance_diagonal)
                ).log_prob(batch_y)
                loss.sum().backward()

            else:
                # estimate = (net(batch_X) - batch_y) / stds
                # loss = functional.l1_loss(estimate, torch.zeros_like(estimate)) #/ torch.abs(torch.mean(batch_y))
                output = net(batch_X)
                loss = functional.l1_loss(output, batch_y)
                trange.set_description(str(loss.detach()))
                loss.backward()

            optimizer.step()

            if stochastic:
                losses.append(loss.sum().detach())
            else:
                losses.append(loss.detach())

            with torch.no_grad():

                val_loss = None
                zero_loss = None

                if stochastic:
                    val_mean, val_covariance_diagonal = net(
                        torch.Tensor(validation_samples["X"])
                    )
                    val_loss = -MultivariateNormal(
                        val_mean, torch.diag_embed(val_covariance_diagonal)
                    ).log_prob(torch.Tensor(validation_samples["y"]))
                    val_losses.append(val_loss.sum().detach().numpy())
                else:
                    # val_estimate = (net(torch.Tensor(validation_samples['X'])) - means) /stds
                    # val_loss = functional.l1_loss(val_estimate, torch.Tensor(validation_samples['y'])) #/ torch.abs(torch.mean(batch_y))
                    val_loss = functional.l1_loss(
                        net(
                            (torch.Tensor(validation_samples["X"]) - X_mean.detach())
                            / X_std.detach()
                        ),
                        (torch.Tensor(validation_samples["y"]) - y_mean.detach())
                        / y_std.detach(),
                    )
                    val_losses.append(val_loss.detach().numpy())

                zero_loss = functional.l1_loss(
                    (torch.Tensor([0 for i in range(10)]) - y_mean.detach())
                    / y_std.detach(),
                    ((torch.Tensor(validation_samples["y"])).detach() - y_mean.detach())
                    / y_std.detach(),
                )  # / torch.abs(torch.mean(batch_y))

                zero_losses.append(zero_loss)
    with torch.no_grad():
        val_diff = (
            net(
                (torch.Tensor(validation_samples["X"]) - X_mean.detach())
                / X_std.detach()
            )
            - (torch.Tensor(validation_samples["y"]) - y_mean.detach()) / y_std.detach()
        )
        val_diff_abs = torch.abs(val_diff)
        avg_abs_diff = torch.mean(val_diff_abs, axis=0)
    return (
        net,
        losses,
        val_losses,
        zero_losses,
        X_mean.detach().numpy(),
        y_mean.detach().numpy(),
        X_std.detach().numpy(),
        y_std.detach().numpy(),
        avg_abs_diff.numpy(),
    )






if __name__ == "__main__":

    if not len(sys.argv) == 2:
        print("usage: ./train_grounding_function.py <params.json>")
        exit(1)




    rl_config = get_json_from_file_name(sys.argv[1])

    IS_CONTROL = rl_config['control']
    NETWORK = export_network.return_network("default")

    os.makedirs(rl_config['output_path'],exist_ok = True)

    if not IS_CONTROL:
        grounding_phase(rl_config)
    # grounded_retraining_phase(rl_config)
    # visualization_phase(rl_config)


   
