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
        / "multi_give_way_export.pt"
    )

    model = trainer.get_policy().model.gnn

    torch.save(model, model_path)


if __name__ == "__main__":
    checkpoint_path = "/home/tg/projects/p3p/HetGPPO/scratch/ray_results/multi_give_way/HetGPPO/MultiPPOTrainer_multi_give_way_d3cfa_00000_0_2024-02-28_16-24-21/checkpoint_000040"
    export(checkpoint_path)
