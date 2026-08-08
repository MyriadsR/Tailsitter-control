"""DATT-style temporal convolution applied only to future references."""

# Load Conda's NumPy runtime before PyTorch selects its bundled C++ libraries.
import numpy as _numpy  # noqa: F401
import torch
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class PreviewFeaturesExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space: spaces.Dict, features_dim=64):
        super().__init__(observation_space, features_dim)
        state_dim = observation_space["state"].shape[0]
        preview_channels, preview_points = observation_space["preview"].shape
        if preview_points < 5:
            raise ValueError("Conv1d 预瞄序列至少需要 5 个点")
        self.preview_net = torch.nn.Sequential(
            torch.nn.Conv1d(preview_channels, 32, kernel_size=3),
            torch.nn.ReLU(),
            torch.nn.Conv1d(32, 16, kernel_size=3),
            torch.nn.ReLU(),
            torch.nn.Flatten(),
        )
        preview_dim = 16 * (preview_points - 4)
        self.state_net = torch.nn.Sequential(
            torch.nn.Linear(state_dim, 32),
            torch.nn.ReLU(),
        )
        self.fusion = torch.nn.Sequential(
            torch.nn.Linear(preview_dim + 32, features_dim),
            torch.nn.ReLU(),
        )

    def forward(self, observations):
        preview = self.preview_net(observations["preview"])
        state = self.state_net(observations["state"])
        return self.fusion(torch.cat([state, preview], dim=1))
