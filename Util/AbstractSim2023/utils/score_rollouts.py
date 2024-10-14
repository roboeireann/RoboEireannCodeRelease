# derived from https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html

import numpy as np
import json
from scipy.stats import multivariate_normal
import math
from threading import Thread
import socket
import os.path
import time
import sys
sys.path.append(sys.path[0] + "/..")
import copy


import matplotlib.pyplot as plt
import matplotlib.colors as mcolors


try:
    from scipy.stats import bootstrap
except:
    print("bootstrap unavailable, skipping import")


from utils.utils import get_json_from_file_name
from envs.push_ball_to_goal import PushBallToGoalEnv
from utils.merge_trajectories import merge_trajectories

EPISODE_LENGTH = 800

# https://en.wikipedia.org/wiki/Binomial_proportion_confidence_interval
def binomial_confidence_interval(data, confidence=0.95):

    z = (1 + confidence) / 2
    p_hat = sum(data) / len(data)
    upper = p_hat + (z * math.sqrt((p_hat * (1 - p_hat)) / (len(data))))
    lower = p_hat - (z * math.sqrt((p_hat * (1 - p_hat)) / (len(data))))
    return lower, p_hat, upper


def average_x_goal_distance(trajectories):
    episode_count = len(trajectories["observations"]) / EPISODE_LENGTH
    print(episode_count)
    episode_count = int(episode_count)
    averages = []

    for i in range(episode_count):
        total = 0
        for c in range(EPISODE_LENGTH):
            observation = trajectories["observations"][i * EPISODE_LENGTH + c]
            ball_x = -4500 - (observation[2] * 9000)
            total += ball_x
        total = total / EPISODE_LENGTH
        averages.append(total)
    print(averages)
    print(sum(averages) / len(averages))


def plot_episode_starts(trajectories, n_starts=100):
    env = PushBallToGoalEnv()
    x_list = []
    y_list = []
    complete = False
    for i in range(len(trajectories["abstract_states"])):
        if trajectories["episode_starts"][i] == True:
            env.set_abstract_state(trajectories["abstract_states"][i])
            x_list.append(env.target_x)
            y_list.append(env.target_y)
        if len(x_list) == n_starts and len(y_list) == n_starts:
            complete = True
            break
    if complete == False:
        print(f"failed to collect {n_starts} x,y pairs")
        exit(1)
    print(x_list)
    print(y_list)
    plt.scatter(x_list, y_list, c=[i for i in range(len(x_list))], cmap="gray")
    return x_list, y_list


def split_episodes(trajectories):
    episodes = []
    episode = {"abstract_states": [], "actions": []}
    for i in range(len(trajectories["abstract_states"])):
        if i != 0 and trajectories["episode_starts"][i]:
            episodes.append(episode)
            episode = {"abstract_states": [], "actions": []}
            episode["abstract_states"].append(trajectories["abstract_states"][i])
            episode["actions"].append(trajectories["actions"][i])
        else:
            episode["abstract_states"].append(trajectories["abstract_states"][i])
            episode["actions"].append(trajectories["actions"][i])
        if i == len(trajectories["abstract_states"]) - 1:
            episodes.append(episode)
    for episode in episodes:
        assert len(episode["abstract_states"]) <= 800
    return episodes


def plot_episode(episode):
    env = PushBallToGoalEnv()
    x = []
    y = []
    target_x = []
    target_y = []
    for i in range(len(episode["abstract_states"])):
        env.set_abstract_state(episode["abstract_states"][i])
        x.append(env.robot_x)
        y.append(env.robot_y)
        target_x.append(env.target_x)
        target_y.append(env.target_y)
    c = [i for i in range(len(episode["abstract_states"]))]
    plt.scatter(x, y, c=c, cmap="Blues")
    plt.scatter(target_x, target_y, c=c, cmap="Oranges")
    


def average_goal_chance(trajectories):
    episode_count = len(trajectories["observations"]) / EPISODE_LENGTH
    print(episode_count)
    episode_count = int(episode_count)
    averages = []

    for i in range(episode_count):
        goal = 0
        for c in range(EPISODE_LENGTH):
            observation = trajectories["observations"][i * EPISODE_LENGTH + c]
            ball_x = -4500 - (observation[2] * 9000)
            ball_y = 0 - (observation[3] * 6000)

            if (
                math.sqrt(
                    (observation[2] * 9000) ** 2 + (((observation[3] * 6000) ** 2))
                )
                < 600
            ):
                goal = 1
                continue

        averages.append(goal)
    print(averages)
    print(sum(averages) / len(averages))

    return averages


def average_action_scale(trajectories, first_n, title=None):
    action_scales = []
    episode_counter = 0
    for i in range(len(trajectories["actions"])):
        # print(i)
        # print(trajectories["actions"][i])
        action_scales.append(
            math.sqrt(
                trajectories["actions"][i][1] ** 2 + trajectories["actions"][i][2] ** 2
            )
        )
        if i != 0 and trajectories["episode_starts"][i] == 1:
            episode_counter += 1
            if episode_counter == first_n:
                break
    average = sum(action_scales) / len(action_scales)
    print(average)
    plt.hist(action_scales)
    if title != None:
        plt.title(title)
    


def new_average_goal_chance(trajectories, first_n, title=None):
    episode_lengths = []
    length_counter = 0
    for i in range(len(trajectories["observations"])):
        print(i)
        if (i != 0 and trajectories["episode_starts"][i] == 1) or i == len(trajectories['observations'])-1:
            episode_lengths.append(length_counter)
            length_counter = 0
        length_counter += 1
        
    print(episode_lengths[:100])
    print(len(episode_lengths[:100]))
    episode_lengths = episode_lengths[:100]
    successes = [(1 if x < 800 else 0) for x in episode_lengths]
    percent = sum(successes) / len(successes)
    print(percent)
    assert len(episode_lengths) == 100
    plt.hist(episode_lengths)
    if title != None:
        plt.title(title)
        plt.savefig(FIGURE_PATH + f"/{title}.png")

    
    return episode_lengths


if __name__ == "__main__":
    if not len(sys.argv) == 2:
        print("usage: score_rollouts.py <params.json>")



    rl_config = get_json_from_file_name(sys.argv[1])




    FIGURE_PATH = f"{rl_config['output_path']}/figures"

    os.makedirs(FIGURE_PATH,exist_ok = True)




    # test = get_json_from_file_name("test_trajectories.jso")
    # episodes = split_episodes(test)
    # print(len(episodes))

    # merged_grounding_trajectories = get_json_from_file_name("merged_trajectories.json")
    # merged_trained_trajectories = get_json_from_file_name("merged_grounded_trajectories.json")
    # f1 = plt.figure()
    # plot_episode_starts(merged_trained_trajectories, n_starts=100)
    # f2 = plt.figure()
    # plot_episode_starts(merged_grounding_trajectories, n_starts=100)
    # 

    # merged_trajectories = get_json_from_file_name("old_merged_trajectories.json")
    # average_action_scale(merged_trajectories, 100, title= "Euclidean Action Scale pre-grounding")

    # merged_trajectories = get_json_from_file_name("old_merged_trained_trajectories.json")
    # average_action_scale(merged_trajectories, 100, title= "Euclidean Action Scale post-grounding")

    # exit()


    merge_trajectories(rl_config['trajectories_source'], f"{rl_config['output_path']}/merged_trajectories.json", 1.0)

    merged_grounding_trajectories = get_json_from_file_name(f"{rl_config['output_path']}/merged_trajectories.json")
    #merged_trained_trajectories = get_json_from_file_name(
    #    "merged_grounded_trajectories.json"
    #)
    merge_trajectories(f"{rl_config['output_path']}/result_robosoccer", f"{rl_config['output_path']}/merged_trained_trajectories.json",1.0)



    merged_trained_trajectories = get_json_from_file_name(f"{rl_config['output_path']}/merged_trained_trajectories.json")

    # average_action_scale(merged_grounding_trajectories, 100, title= "Euclidean Action Scale pre-grounding")
    # average_action_scale(merged_trained_trajectories, 100, title= "Euclidean Action Scale post-grounding")

    """
    episodes = split_episodes(merged_grounding_trajectories)
    for episode in episodes:
        plot_episode(episode)
    exit()
    """

    f1 = plt.figure()
    x_list, y_list = plot_episode_starts(merged_trained_trajectories, n_starts=100)
    plt.savefig(FIGURE_PATH + "/grounding_trajectory_starts.png")

    f2 = plt.figure()
    plot_episode_starts(merged_grounding_trajectories, n_starts=100)
    plt.savefig(FIGURE_PATH + "/grounded_trajectory_starts.png")

    
    f3 = plt.figure()
    grounding_episode_lengths = new_average_goal_chance(
        merged_grounding_trajectories,
        100,
        title="Episode Length Distribution pre-grounding",
    )
    f4 = plt.figure()
    grounded_episode_lengths = new_average_goal_chance(
        merged_trained_trajectories,
        100,
        title="Episode Length Distribution post-grounding",
    )

    f5 = plt.figure()
    diffs = [
        grounded - grounding
        for grounded, grounding in zip(
            grounded_episode_lengths, grounding_episode_lengths
        )
    ]
    plt.hist(diffs)
    plt.savefig(FIGURE_PATH + "/grounding_diff_hist.png")

    # normed_diffs = (np.array(diffs)-np.mean(np.array(diffs)))/np.std(np.array(diffs))
    f6 = plt.figure()
    plt.scatter(
        x_list,
        y_list,
        norm=mcolors.CenteredNorm(halfrange=5000),
        c=[-diff for diff in list(diffs)],
        cmap="RdYlGn",
    )
    plt.savefig(FIGURE_PATH + "/positional_grounding_diff_scatter.png")

    

    print(grounding_episode_lengths)
    print(grounded_episode_lengths)

    print("grounding episode length confidence interval")
    grounding_res = bootstrap(
        (grounding_episode_lengths,),
        np.mean,
        confidence_level=0.95,
    )
    
    print(grounding_res.confidence_interval)

    print("grounded episode length confidence interval")
    grounded_res = bootstrap(
        (grounded_episode_lengths,),
        np.mean,
        confidence_level=0.95,
    )
    print(grounded_res.confidence_interval)

    labels = ["pre-grounded", "grounded"]

    grounding_mean = np.mean(grounding_episode_lengths)
    grounded_mean = np.mean(grounded_episode_lengths)
    print(grounding_mean)
    print(grounded_mean)

    fig, ax = plt.subplots()
    # ax.bar([1,2,3], [output_stats[0],output_grounded_stats[0], output_target_stats[0]], yerr = [output_stats[1],output_grounded_stats[1],output_target_stats[1]], color = 'aquamarine')
    ax.bar(
        [1, 2],
        [grounding_mean, grounded_mean],
        yerr=[
            [
                grounding_mean - grounding_res.confidence_interval.low,
                grounded_mean - grounded_res.confidence_interval.low,
            ],
            [
                grounding_res.confidence_interval.high - grounding_mean,
                grounded_res.confidence_interval.high - grounded_mean,
            ],
        ],
        color="aquamarine",
    )

    # ax.set_xticks([1,2,3])
    # ax.set_xticklabels(labels)

    ax.set_xticks([1, 2])
    ax.set_xticklabels(labels)

    ax.set_ylabel("Average Episode Length n=100")
    ax.set_title(
        "95\% bootstrap confidence interval for average episode length (n=100)"
    )
    plt.legend()
    plt.savefig(FIGURE_PATH + "/episode_length_mean_confidence_interval.png")

    

    print("grounding episode length stdev confidence interval")
    grounding_res = bootstrap(
        (grounding_episode_lengths,),
        np.std,
        confidence_level=0.95,
    )
    print(grounding_res.confidence_interval)

    print("grounded episode length stdev confidence interval")
    grounded_res = bootstrap(
        (grounded_episode_lengths,),
        np.std,
        confidence_level=0.95,
    )
    print(grounded_res.confidence_interval)

    labels = ["pre-grounded", "grounded"]

    grounding_mean = np.std(grounding_episode_lengths)
    grounded_mean = np.std(grounded_episode_lengths)
    print(grounding_mean)
    print(grounded_mean)

    fig, ax = plt.subplots()
    # ax.bar([1,2,3], [output_stats[0],output_grounded_stats[0], output_target_stats[0]], yerr = [output_stats[1],output_grounded_stats[1],output_target_stats[1]], color = 'aquamarine')
    ax.bar(
        [1, 2],
        [grounding_mean, grounded_mean],
        yerr=[
            [
                grounding_mean - grounding_res.confidence_interval.low,
                grounded_mean - grounded_res.confidence_interval.low,
            ],
            [
                grounding_res.confidence_interval.high - grounding_mean,
                grounded_res.confidence_interval.high - grounded_mean,
            ],
        ],
        color="aquamarine",
    )

    # ax.set_xticks([1,2,3])
    # ax.set_xticklabels(labels)

    ax.set_xticks([1, 2])
    ax.set_xticklabels(labels)

    ax.set_ylabel("Average Episode Length std n=100")
    ax.set_title(
        "95\% bootstrap confidence interval for average episode length standard deviation (n=100)"
    )
    plt.legend()
    plt.savefig(FIGURE_PATH + "/episode_length_stdev_confidence_interval.png")
    

    # normed_diffs = (np.array(diffs)-np.mean(np.array(diffs)))/np.std(np.array(diffs))

    f7 = plt.figure()
    plt.scatter(
        x_list,
        y_list,
        norm=mcolors.CenteredNorm(halfrange=1),
        c=[(1 if x < 800 else -1) for x in grounding_episode_lengths],
        cmap="RdYlGn",
    )
    plt.savefig(FIGURE_PATH + "/grounding_episodes_success_scatter.png")





    # average_x_goal_distance(trajectories)
    # result = average_goal_chance(trajectories)
    # print(binomial_confidence_interval(result))
