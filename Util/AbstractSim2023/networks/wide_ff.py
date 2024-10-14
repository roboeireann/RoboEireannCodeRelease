import torch.nn as nn
import torch.nn.functional as functional

class Network(nn.Module):
    def __init__(self, history_length):
        super().__init__()
        self.dense1 = nn.Linear(10 * (history_length + 1) + 3 * history_length, 2048)
        self.dense2 = nn.Linear(2048, 1024)
        self.dense3 = nn.Linear(1024, 10)

    def forward(self, x):
        x = functional.relu(self.dense1(x))
        x = functional.relu(self.dense2(x))
        x = self.dense3(x)
        return x