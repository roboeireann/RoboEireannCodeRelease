
import sys
sys.path.append(sys.path[0] + "/..")


import matplotlib.pyplot as plt
import pickle
import torch


from utils.score_rollouts import split_episodes
from utils.utils import get_json_from_file_name
from utils.train_grounding_function import ENVIRONMENTS
from networks import export_network
from envs.grounded_env import GroundedEnv

NETWORK = []

def analyze_grounding_function(rl_config):

    #get episodes

    trajectories = get_json_from_file_name(f"{rl_config['output_path']}/merged_trajectories.json")
    episodes = split_episodes(trajectories)

    HISTORY_BUFFER_LENGTH = rl_config["history_length"] + 1
    ENVIRONMENT = ENVIRONMENTS[rl_config['environment']]
    DISPLAY_STEPS = rl_config['display_steps']
    SHOW_BALL = rl_config['show_ball']

    for episode in episodes:
        env = ENVIRONMENT()

        episode["abstract_states"] = episode["abstract_states"][:DISPLAY_STEPS]
        episode["actions"] = episode["actions"][:DISPLAY_STEPS]

        # first, look at the recorded episode
        x_coords = []
        y_coords = []
        target_x_coords = []
        target_y_coords = []
        for state in episode["abstract_states"]:

            env.set_abstract_state(state)

            x_coords.append(env.robot_x)
            y_coords.append(env.robot_y)
            target_x_coords.append(env.target_x)
            target_y_coords.append(env.target_y)

        plt.figure(1)
        plt.title("Recorded Episode States")
        plt.scatter(
            x_coords, y_coords, c=[i for i in range(len(x_coords))], cmap="plasma"
        )
        if SHOW_BALL:
            plt.scatter(
                target_x_coords,
                target_y_coords,
                c=[i for i in range(len(target_x_coords))],
                cmap="Greens",
            )
        

        # next, we look at the recorded actions taken in abstractsim with no grounding

        x_coords = []
        y_coords = []
        target_x_coords = []
        target_y_coords = []

        env.set_abstract_state(episode['abstract_states'][0])
        x_coords.append(env.robot_x)
        y_coords.append(env.robot_y)
        target_x_coords.append(env.target_x)
        target_y_coords.append(env.target_y)
    
        for action in episode["actions"]:

            env.step(action)

            x_coords.append(env.robot_x)
            y_coords.append(env.robot_y)
            target_x_coords.append(env.target_x)
            target_y_coords.append(env.target_y)

        plt.figure(2)
        plt.title("Non-grounded Recorded Action Rollout in Abstract Env")
        plt.scatter(
            x_coords, y_coords, c=[i for i in range(len(x_coords))], cmap="plasma"
        )
        if SHOW_BALL:
            plt.scatter(
                target_x_coords,
                target_y_coords,
                c=[i for i in range(len(target_x_coords))],
                cmap="Greens",
            )

        #finally, we look at the predicted rollout for the actions and initial state with grounding

    
        standardization_info = None
        with open(f"{rl_config['output_path']}/standardization_info.pkl", 'rb') as standardization_info_file:
            standardization_info = pickle.load(standardization_info_file)
        net = NETWORK(rl_config["history_length"])
        net.load_state_dict(torch.load(f"{rl_config['output_path']}/sim_grounding_function"))
        net.training = False

        env = GroundedEnv(env,net,standardization_info,rl_config["history_length"])
        


        x_coords = []
        y_coords = []
        target_x_coords = []
        target_y_coords = []

        env.reset(initial_state = episode['abstract_states'][0])
        x_coords.append(env.env.robot_x)
        y_coords.append(env.env.robot_y)
        target_x_coords.append(env.env.target_x)
        target_y_coords.append(env.env.target_y)
    
        for action in episode["actions"]:

            env.step(action)

            x_coords.append(env.env.robot_x)
            y_coords.append(env.env.robot_y)
            target_x_coords.append(env.env.target_x)
            target_y_coords.append(env.env.target_y)

        plt.figure(3)
        plt.title("Grounded Recorded Action Rollout in Abstract Env")
        plt.scatter(
            x_coords, y_coords, c=[i for i in range(len(x_coords))], cmap="plasma"
        )
        if SHOW_BALL:
            plt.scatter(
                target_x_coords,
                target_y_coords,
                c=[i for i in range(len(target_x_coords))],
                cmap="Greens",
            )

        env = ENVIRONMENT()

        predicted_x_coords = []
        predicted_y_coords = []
        predicted_target_x_coords = []
        predicted_target_y_coords = []

        default_predicted_x_coords = []
        default_predicted_y_coords = []
        default_predicted_target_x_coords = []
        default_predicted_target_y_coords = []

        for i in range(len(episode["abstract_states"]) - HISTORY_BUFFER_LENGTH):

            observations = episode["abstract_states"]
            actions = episode["actions"]

            sample_observations = observations[i : i + HISTORY_BUFFER_LENGTH]
            sample_actions = actions[i : i + HISTORY_BUFFER_LENGTH - 1]

            abstract_state = sample_observations[-2]
            # print("abstract_state")
            # print(abstract_state)
            env.set_abstract_state(abstract_state)
            # print("after set")
            # print(env.get_abstract_state())
            env.step(sample_actions[-1])

            default_predicted_x_coords.append(env.robot_x)
            default_predicted_y_coords.append(env.robot_y)
            default_predicted_target_x_coords.append(env.target_x)
            default_predicted_target_y_coords.append(env.target_y)

            true_final_state = sample_observations[-1]
            abstract_sim_final_state = env.get_abstract_state().tolist()
            sample_observations[-1] = abstract_sim_final_state
            # print("sample_observations2")
            # print(sample_observations)
            # print(sample_actions)
            # print("diff")
            # print(true_final_state)
            # print(abstract_sim_final_state)
            assert len(true_final_state) == 10
            assert len(abstract_sim_final_state) == 10
            # if max(y) > .1 or min(y) < -.1:
            #    continue

            grounding_input = [
                float(i)
                for table in [sample_observations, sample_actions]
                for entry in table
                for i in entry
            ]
            print(type(grounding_input))
            print(type(standardization_info["X_mean"]))
            grounding_input = (
                torch.Tensor(grounding_input)
                - torch.Tensor(standardization_info["X_mean"])
            ) / torch.Tensor(standardization_info["X_std"])
            offset = net(grounding_input).detach().numpy()

            offset = (
                (torch.Tensor(offset) * torch.Tensor(standardization_info["y_std"]))
                + standardization_info["y_mean"]
            ).numpy()

            new_state = tuple(
                [
                    state_entry + offset_entry
                    for state_entry, offset_entry in zip(
                        abstract_sim_final_state, offset
                    )
                ]
            )
            env.set_abstract_state(new_state)
            predicted_x_coords.append(env.robot_x)
            predicted_y_coords.append(env.robot_y)
            predicted_target_x_coords.append(env.target_x)
            predicted_target_y_coords.append(env.target_y)
        print(predicted_x_coords)
       
        plt.figure(4)
        plt.title("AbstractSim Default Predictions From Recorded Preceding States/Actions")

        plt.scatter(
            default_predicted_x_coords,
            default_predicted_y_coords,
            c=[i for i in range(len(predicted_x_coords))],
            cmap="plasma",
        )
        if SHOW_BALL:
            plt.scatter(
                default_predicted_target_x_coords,
                default_predicted_target_y_coords,
                c=[i for i in range(len(predicted_target_x_coords))],
                cmap="Greens",
            )

        plt.figure(5)
        plt.title("AbstractSim Grounded Predictions From Recorded Preceding States/Actions")

        plt.scatter(
            predicted_x_coords,
            predicted_y_coords,
            c=[i for i in range(len(predicted_x_coords))],
            cmap="plasma",
        )
        if SHOW_BALL:
            plt.scatter(
                predicted_target_x_coords,
                predicted_target_y_coords,
                c=[i for i in range(len(predicted_target_x_coords))],
                cmap="Greens",
            )
        plt.show()


if __name__ == "__main__":

    if not len(sys.argv) == 2:
        print("usage: ./analyze_grounding_function.py <params.json>")
        exit(1)


    rl_config = get_json_from_file_name(sys.argv[1])

    IS_CONTROL = rl_config['control']

    if IS_CONTROL:
        print("This analysis requires a trained grounding function, these parameters refer to a control run")
        exit(1)
    
    NETWORK = export_network.return_network("default")

    analyze_grounding_function(rl_config)
    