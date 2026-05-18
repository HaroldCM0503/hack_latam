import torch
import torch.nn as nn

class CSIAnomalyDetector(nn.Module):
    """
    A 1D CNN Multi-Task model for detecting transits and estimating speed
    from a temporal sequence of CSI amplitudes.
    """
    def __init__(self, input_size: int, cnn_channels: tuple, fc_hidden: int, dropout_rate: float = 0.3):
        super().__init__()
        
        # 1D CNN Feature Extractor
        layers = []
        in_channels = 1 # Single channel sequence
        
        for out_channels in cnn_channels:
            layers.append(nn.Conv1d(in_channels, out_channels, kernel_size=5, padding=2))
            layers.append(nn.ReLU())
            layers.append(nn.MaxPool1d(kernel_size=2))
            in_channels = out_channels
            
        self.feature_extractor = nn.Sequential(*layers)
        
        # Calculate flattened size
        seq_len = input_size
        for _ in cnn_channels:
            seq_len = seq_len // 2
        flattened_size = in_channels * seq_len
        
        # Shared fully connected
        self.shared_fc = nn.Sequential(
            nn.Linear(flattened_size, fc_hidden),
            nn.LayerNorm(fc_hidden),
            nn.ReLU(),
            nn.Dropout(dropout_rate)
        )
        
        # Head 1: Detection (Is Transit?)
        self.transit_head = nn.Sequential(
            nn.Linear(fc_hidden, 1),
            nn.Sigmoid()
        )
        
        # Head 2: Regression (Speed)
        self.speed_head = nn.Sequential(
            nn.Linear(fc_hidden, 1),
            nn.ReLU() # Speed is always non-negative
        )
        
    def forward(self, x):
        """
        Forward pass.
        Args:
            x (torch.Tensor): Input tensor of shape (batch_size, input_size).
        Returns:
            transit_prob (torch.Tensor): shape (batch_size, 1)
            speed (torch.Tensor): shape (batch_size, 1)
        """
        # Conv1d expects (batch_size, channels, sequence_length)
        x = x.unsqueeze(1) 
        
        features = self.feature_extractor(x)
        features = features.view(features.size(0), -1) # Flatten
        
        shared = self.shared_fc(features)
        
        transit_prob = self.transit_head(shared)
        speed = self.speed_head(shared)
        
        return transit_prob, speed

    @classmethod
    def from_config(cls, config):
        return cls(
            input_size=config.input_size,
            cnn_channels=config.cnn_channels,
            fc_hidden=config.fc_hidden,
            dropout_rate=config.dropout_rate
        )
