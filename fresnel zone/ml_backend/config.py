import os
from dataclasses import dataclass

@dataclass
class TrainingConfig:
    # Model architecture
    input_size: int = 500  # 500 time-domain amplitude samples
    cnn_channels: tuple = (16, 32, 64)
    fc_hidden: int = 128
    dropout_rate: float = 0.3

    # Training hyperparameters
    batch_size: int = 32
    learning_rate: float = 1e-3
    num_epochs: int = 50
    weight_decay: float = 1e-5

    # Paths
    checkpoint_dir: str = "checkpoints"
    best_model_name: str = "best_model_v3.pth"

    def __post_init__(self):
        # Ensure checkpoint directory exists
        os.makedirs(self.checkpoint_dir, exist_ok=True)
