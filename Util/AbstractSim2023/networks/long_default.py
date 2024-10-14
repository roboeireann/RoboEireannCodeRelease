import torch.nn as nn
import torch.nn.functional as functional

class Network(nn.Module):
    def __init__(self, history_length):
        super().__init__()
        self.dense1 = nn.Linear(10 * (history_length + 1) + 3 * history_length, 100)
        self.dense2 = nn.Linear(100, 100)
        self.dense3 = nn.Linear(100, 50)
        self.dense4 = nn.Linear(50, 50)
        self.dense5 = nn.Linear(50, 50)
        self.dense6 = nn.Linear(50, 50)
        self.dense7 = nn.Linear(50, 50)
        self.dense8 = nn.Linear(50, 50)
        self.dense9 = nn.Linear(50, 10)

    def forward(self, x):
        x = functional.relu(self.dense1(x))
        x = functional.relu(self.dense2(x))
        x = functional.relu(self.dense3(x))
        x = functional.relu(self.dense4(x))
        x = functional.relu(self.dense5(x))
        x = functional.relu(self.dense6(x))
        x = functional.relu(self.dense7(x))
        x = functional.relu(self.dense8(x))
        x = self.dense9(x)
        return x