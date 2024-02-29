#  Copyright (c) 2022.
#  ProrokLab (https://www.proroklab.org/)
#  All rights reserved.

from pathlib import Path
from typing import Union

import torch

from utils import EvaluationUtils


def export(
    checkpoint_path: Union[str, Path],
):

    config, trainer, env = EvaluationUtils.get_config_trainer_and_env_from_checkpoint(
        checkpoint_path
    )

    model_path = (
        Path(checkpoint_path).parent
        / "give_way_export.pt"
    )

    model = trainer.get_policy().model.gnn

    torch.save(model, model_path)


if __name__ == "__main__":
    checkpoint_path = "/home/tg/projects/p3p/HetGPPO/scratch/ray_results/give_way/HetGPPO/MultiPPOTrainer_give_way_4b993_00000_0_2024-02-28_21-28-20/checkpoint_000407"
    export(checkpoint_path)
