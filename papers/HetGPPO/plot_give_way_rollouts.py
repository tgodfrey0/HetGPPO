#  Copyright (c) 2022-2023.
#  ProrokLab (https://www.proroklab.org/)
#  All rights reserved.

from pathlib import Path
from typing import Set, Union, List

import numpy as np
import tikzplotlib
import torch
from matplotlib import pyplot as plt

from evaluate.evaluate_model import compute_action_corridor
from utils import InjectMode, EvaluationUtils


def success_from_x_position(x1, x2):
    distance_from_goal1 = torch.minimum(x1 - 2, torch.zeros_like(x1)).abs()
    distance_from_goal2 = torch.maximum(2 + x2, torch.zeros_like(x2)).abs()
    distance = distance_from_goal1 + distance_from_goal2  # 0 to 8
    distance /= -8
    distance += 1
    return distance


def get_distance(
    checkpoint_path: Union[str, Path],
    het: bool,
    n_episodes: int,
    agents_to_inject: Set = None,
    inject_mode: InjectMode = None,
    noise_delta: float = None,
):
    def update_config(config):
        config["env_config"]["scenario_config"]["obs_noise"] = 0
        return config

    config, trainer, env = EvaluationUtils.get_config_trainer_and_env_from_checkpoint(
        checkpoint_path, config_update_fn=update_config
    )

    inject = agents_to_inject is not None and len(agents_to_inject) > 0

    def action_callback(observation):
        model = torch.load(
            f"/Users/Matteo/Downloads/{'het_' if het else ''}a_range_1_u_range_0_5_[3_2_0_002]_0_05_dt_0_1_friction_0_dt_delay_option_0.pt"
        )
        model.eval()

        action = compute_action_corridor(
            pos0_x=observation[0][0],
            pos0_y=observation[0][1],
            vel0_x=observation[0][2],
            vel0_y=observation[0][3],
            pos1_x=observation[1][0],
            pos1_y=observation[1][1],
            vel1_x=observation[1][2],
            vel1_y=observation[1][3],
            model=model,
            u_range=0.5,
        )
        return action

    rewards, _, obs, actions = EvaluationUtils.rollout_episodes(
        n_episodes=n_episodes,
        render=False,
        get_obs=True,
        get_actions=True,
        trainer=None,
        action_callback=action_callback,
        env=env,
        inject=inject,
        inject_mode=inject_mode,
        agents_to_inject=agents_to_inject,
        noise_delta=noise_delta,
    )

    obs = torch.tensor(np.array(obs))
    obs = obs.view(obs.shape[0], -1, env.env.n_agents, 6)
    pos_x = 0
    x_positions = obs[..., pos_x]

    x_positions_agent_0 = x_positions[..., 0]
    x_positions_agent_1 = x_positions[..., 1]

    distance = success_from_x_position(x_positions_agent_0, x_positions_agent_1)

    # Cut after success
    argmax = torch.argmax(distance, dim=1)
    max_argmax = int(torch.max(argmax).item())
    distance = distance[..., :max_argmax]

    return distance


def evaluate_increasing_noise(
    model_paths: List[Union[str, Path]],
    envs,
    n_episodes_per_model: int,
    agents_to_inject: Set,
    inject_mode: InjectMode,
    noise_data_points,
    noise_range,
):
    assert len(noise_range) == 2
    noises = np.linspace(noise_range[0], noise_range[1], noise_data_points)
    completions = np.zeros(
        (
            len(model_paths),
            len(noises),
            n_episodes_per_model,
        )
    )

    for j, noise in enumerate(noises):
        for i, model_path in enumerate(model_paths):

            def action_callback(observation):
                model = torch.load(model_path)
                model.eval()

                action = compute_action_corridor(
                    pos0_x=observation[0][0],
                    pos0_y=observation[0][1],
                    vel0_x=observation[0][2],
                    vel0_y=observation[0][3],
                    pos1_x=observation[1][0],
                    pos1_y=observation[1][1],
                    vel1_x=observation[1][2],
                    vel1_y=observation[1][3],
                    model=model,
                    u_range=0.5,
                )
                return action

            rews, _, obs, _ = EvaluationUtils.rollout_episodes(
                n_episodes=n_episodes_per_model,
                render=True,
                get_obs=True,
                get_actions=False,
                trainer=None,
                action_callback=action_callback,
                env=envs[i],
                inject=True,
                inject_mode=inject_mode,
                agents_to_inject=agents_to_inject,
                noise_delta=noise,
            )

            obs = torch.tensor(np.array(obs))
            obs = obs.view(obs.shape[0], -1, envs[i].env.n_agents, 6)
            pos_x = 0
            x_positions = obs[..., pos_x]
            x_positions_agent_0 = x_positions[..., 0]
            x_positions_agent_1 = x_positions[..., 1]
            distance = success_from_x_position(x_positions_agent_0, x_positions_agent_1)

            # Cut after success
            max_distance = distance.max(dim=-1)[0]

            completions[i, j] = max_distance

    fig, ax = plt.subplots(figsize=(5, 5))
    CB_color_cycle = [
        "#377eb8",
        "#ff7f00",
        "#4daf4a",
        "#f781bf",
        "#a65628",
        "#984ea3",
        "#999999",
        "#e41a1c",
        "#dede00",
    ]
    ax.grid()
    for model_num, trainer in enumerate(model_paths):

        to_plot = completions
        # to_plot = done

        mean = to_plot[model_num].mean(1)
        std = to_plot[model_num].std(1)
        model_title = "HetGPPO" if model_num == 0 else "GPPO"
        (mean_line,) = ax.plot(
            noises, mean, label=model_title, color=CB_color_cycle[model_num]
        )
        ax.fill_between(
            noises,
            mean + std,
            mean - std,
            color=mean_line.get_color(),
            alpha=0.3,
        )
    ax.set_xlabel("Uniform observation noise")
    ax.set_ylabel("Reward")
    ax.legend()

    # tikzplotlib.save(
    #     f"trial.tex",
    #     textsize=18,
    # )
    # plt.savefig(f"trial.pdf", bbox_inches="tight", pad_inches=0)
    plt.show()


def plot_distances(het_distance, homo_distance):
    tex_fonts = {
        # Use LaTeX to write all text
        "text.usetex": True,
        "text.latex.preamble": "\\usepackage{libertine}\n\\usepackage[libertine]{newtxmath}",
        "font.family": "Linux Libertine",
        # Use 10pt font in plots, to match 10pt font in document
        "axes.labelsize": 19,
        "font.size": 10,
        # Make the legend/label fonts a little smaller
        "legend.fontsize": 16,
        "legend.title_fontsize": 7,
        "xtick.labelsize": 16,
        "ytick.labelsize": 16,
    }

    plt.rcParams.update(tex_fonts)

    fig, ax = plt.subplots(figsize=(5, 5))

    CB_color_cycle = [
        "#377eb8",
        "#ff7f00",
        "#4daf4a",
        "#f781bf",
        "#a65628",
        "#984ea3",
        "#999999",
        "#e41a1c",
        "#dede00",
    ]

    for i, episode_obs in enumerate(het_distance):
        completion = np.array(episode_obs)
        max_completion_index = np.argmax(completion) + 1

        completion = completion[:max_completion_index]

        ax.plot(
            np.linspace(0, max_completion_index * 0.05, max_completion_index),
            completion,
            label="HetGPPO" if i == 0 else None,
            color=CB_color_cycle[0],
        )
    for i, episode_obs in enumerate(homo_distance):
        completion = np.array(episode_obs)
        max_completion_index = np.argmax(completion) + 1

        completion = completion[:max_completion_index]

        ax.plot(
            np.linspace(0, max_completion_index * 0.05, max_completion_index),
            completion,
            label="GPPO" if i == 0 else None,
            color=CB_color_cycle[1],
        )

    ax.grid()
    ax.set_xlabel("Seconds")
    ax.set_ylabel("Task completion")
    ax.legend()

    tikzplotlib.save(
        f"trial.tex",
        textsize=18,
    )
    plt.savefig(f"trial.pdf", bbox_inches="tight", pad_inches=0)
    plt.show()


def evaluate_completion():
    checkpoint_path = "/Users/Matteo/Downloads/MultiPPOTrainer_give_way_deploy_72a9b_00000_0_2022-10-18_11-13-01/checkpoint_001294/checkpoint-1294"
    n_episodes = 10

    het_distance = get_distance(checkpoint_path, n_episodes=n_episodes, het=True)
    homo_distance = get_distance(checkpoint_path, n_episodes=n_episodes, het=False)
    plot_distances(het_distance=het_distance, homo_distance=homo_distance)


def evaluate_resilience():
    # hetgppo_model_path = "/Users/Matteo/PycharmProjects/HetGPPO/results/real-world-models/het_a_range_1_u_range_0_5_[3_2_0_002]_0_05_dt_0_1_friction_0_dt_delay_option_0.pt"
    # gppo_model_path = "/Users/Matteo/PycharmProjects/HetGPPO/results/real-world-models/a_range_1_u_range_0_5_[3_2_0_002]_0_05_dt_0_1_friction_0_dt_delay_option_0.pt"
    hetgppo_model_path = "scratch/ray_results/give_way/HetGPPO/MultiPPOTrainer_give_way_381aa_00000_0_2024-02-21_17-11-14/checkpoint_000080/give_way_export.pt"
    gppo_model_path = "scratch/ray_results/give_way/HetGPPO/MultiPPOTrainer_give_way_381aa_00000_0_2024-02-21_17-11-14/checkpoint_000080/give_way_export.pt"

    env = EvaluationUtils.get_config_trainer_and_env_from_checkpoint(
        # "/Users/Matteo/PycharmProjects/HetGPPO/results/MultiPPOTrainer_give_way_deploy_72a9b_00000_0_2022-10-18_11-13-01/checkpoint_001295/checkpoint-1295"
        "/home/tg/projects/p3p/HetGPPO/scratch/ray_results/give_way/HetGPPO/MultiPPOTrainer_give_way_381aa_00000_0_2024-02-21_17-11-14/checkpoint_000080/"
    )[2]

    evaluate_increasing_noise(
        model_paths=[hetgppo_model_path, gppo_model_path],
        envs=[env] * 2,
        n_episodes_per_model=10,
        agents_to_inject={0, 1},
        inject_mode=InjectMode.OBS_NOISE,
        noise_data_points=40,
        noise_range=(0, 10),
    )


if __name__ == "__main__":
    evaluate_resilience()
