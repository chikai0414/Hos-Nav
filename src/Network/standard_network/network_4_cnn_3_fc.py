import torch
import torch.nn.functional as F
import torch.optim as optim
import torch.nn as nn
from torch.distributions import Normal
from torch.optim import Adam
def __len__(self):
        return len(self.buffer)

def weights_init_(m):
    if isinstance(m, nn.Linear):
        torch.nn.init.xavier_uniform_(m.weight, gain=1)
        torch.nn.init.constant_(m.bias, 0)

def soft_update(target, source, tau):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)

def hard_update(target, source):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(param.data)

def action_unnormalized(action, high, low):
    action = low + (action + 1.0) * 0.5 * (high - low)
    action = np.clip(action, low, high)
    return action

class QNetwork(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim):
        super(QNetwork, self).__init__()
        # Q1
        self.cov1_q1 = nn.Conv1d(in_channels=1,out_channels=4,kernel_size=5, padding = 2)
        self.pool1_q1 = nn.MaxPool1d(3, stride=3)
        self.cov2_q1 = nn.Conv1d(in_channels=4,out_channels=8,kernel_size=5, padding = 2)
        self.pool2_q1 = nn.MaxPool1d(3, stride=3)
        self.cov3_q1 = nn.Conv1d(in_channels=8,out_channels=16,kernel_size=5, padding = 2)
        self.pool3_q1 = nn.MaxPool1d(3, stride=3)
        self.cov4_q1 = nn.Conv1d(in_channels=16,out_channels=32,kernel_size=5, padding = 2)
        self.pool4_q1 = nn.MaxPool1d(3, stride=3)

        self.linear1_q1 = nn.Linear(128, 32)
        self.linear2_q1 = nn.Linear(32 + 2 + 2 , hidden_dim)
        self.linear3_q1 = nn.Linear(hidden_dim, hidden_dim)
        self.linear4_q1 = nn.Linear(hidden_dim, hidden_dim)
        self.linear5_q1 = nn.Linear(hidden_dim, 1)
        
        # Q2
        self.cov1_q2 = nn.Conv1d(in_channels=1,out_channels=4,kernel_size=5, padding = 2)
        self.pool1_q2 = nn.MaxPool1d(3, stride=3)
        self.cov2_q2 = nn.Conv1d(in_channels=4,out_channels=8,kernel_size=5, padding = 2)
        self.pool2_q2 = nn.MaxPool1d(3, stride=3)
        self.cov3_q2 = nn.Conv1d(in_channels=8,out_channels=16,kernel_size=5, padding = 2)
        self.pool3_q2 = nn.MaxPool1d(3, stride=3)
        self.cov4_q2 = nn.Conv1d(in_channels=16,out_channels=32,kernel_size=5, padding = 2)
        self.pool4_q2 = nn.MaxPool1d(3, stride=3)

        self.linear1_q2 = nn.Linear(128,32)
        self.linear2_q2 = nn.Linear(32 + 2 + 2 , hidden_dim)
        self.linear3_q2 = nn.Linear(hidden_dim, hidden_dim)
        self.linear4_q2 = nn.Linear(hidden_dim, hidden_dim)
        self.linear5_q2 = nn.Linear(hidden_dim, 1)
        
        self.apply(weights_init_)
        
    def forward(self, state, action):
        t = state.reshape(-1).__len__()/362
        state = torch.reshape(state,(t,1,362))
        goal = state[:,:,360:362]
        goal = torch.reshape(goal,(t,2))
        state = state[:,:,:360]

        #x_state_action = torch.cat([state, action], 1)

        x1 = F.relu(self.cov1_q1(state))
        x1 = F.relu(self.pool1_q1(x1))
        x1 = F.relu(self.cov2_q1(x1))
        x1 = F.relu(self.pool2_q1(x1))
        x1 = F.relu(self.cov3_q1(x1))
        x1 = F.relu(self.pool3_q1(x1))
        x1 = F.relu(self.cov4_q1(x1))
        x1 = F.relu(self.pool4_q1(x1))


        x1 = torch.reshape(x1,(t,128))
        x1 = F.relu(self.linear1_q1(x1))

        x1 = torch.cat([x1,action,goal], 1)

        x1 = F.relu(self.linear2_q1(x1))
        x1 = F.relu(self.linear3_q1(x1))
        x1 = F.relu(self.linear4_q1(x1))
        x1 = self.linear5_q1(x1)
        
        x2 = F.relu(self.cov1_q2(state))
        x2 = F.relu(self.pool1_q2(x2))
        x2 = F.relu(self.cov2_q2(x2))
        x2 = F.relu(self.pool2_q2(x2))
        x2 = F.relu(self.cov3_q2(x2))
        x2 = F.relu(self.pool3_q2(x2))
        x2 = F.relu(self.cov4_q2(x2))
        x2 = F.relu(self.pool4_q2(x2))

        x2 = torch.reshape(x2,(t,128))
        x2 = F.relu(self.linear1_q2(x2))

        x2 = torch.cat([x2, action,goal], 1)

        x2 = F.relu(self.linear2_q2(x2))
        x2 = F.relu(self.linear3_q2(x2))
        x2 = F.relu(self.linear4_q2(x2))
        x2 = self.linear5_q2(x2)
        
        return x1, x2
        
class PolicyNetwork(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim, log_std_min=-20, log_std_max=2):
        super(PolicyNetwork, self).__init__()
        
        self.log_std_min = log_std_min
        self.log_std_max = log_std_max
        
        self.cov1 = nn.Conv1d(in_channels=1,out_channels=4,kernel_size=5, padding = 2)
        self.pool1 = nn.MaxPool1d(3, stride=3)
        self.cov2 = nn.Conv1d(in_channels=4,out_channels=8,kernel_size=5, padding = 2)
        self.pool2 = nn.MaxPool1d(3, stride=3)
        self.cov3 = nn.Conv1d(in_channels=8,out_channels=16,kernel_size=5, padding = 2)
        self.pool3 = nn.MaxPool1d(3, stride=3)
        self.cov4 = nn.Conv1d(in_channels=16,out_channels=32,kernel_size=5, padding = 2)
        self.pool4 = nn.MaxPool1d(3, stride=3)


        self.linear1 = nn.Linear(128, 32)

        self.linear2 = nn.Linear(32 + 2 , hidden_dim)
        self.linear3 = nn.Linear(hidden_dim, hidden_dim)

        self.mean_linear = nn.Linear(hidden_dim, action_dim)
        self.log_std_linear = nn.Linear(hidden_dim, action_dim)

        self.apply(weights_init_)

    def forward(self, state):
       
        t = state.reshape(-1).__len__()/362
        state = torch.reshape(state,(t,1,362))

        goal = state[:,:,360:362]
        goal = torch.reshape(goal,(t,2))

        state = state[:,:,:360]
        x = F.relu(self.cov1(state))
        x = F.relu(self.pool1(x))
        x = F.relu(self.cov2(x))
        x = F.relu(self.pool2(x))
        x = F.relu(self.cov3(x))
        x = F.relu(self.pool3(x))
        x = F.relu(self.cov4(x))
        x = F.relu(self.pool4(x))

        x = torch.reshape(x,(t,128))
        x = F.relu(self.linear1(x))

        x = torch.cat([x,goal],1)
        x = F.relu(self.linear2(x))
        x = F.relu(self.linear3(x))
        mean = self.mean_linear(x)
        log_std = self.log_std_linear(x)
        log_std = torch.clamp(log_std, min=self.log_std_min, max=self.log_std_max)
        return mean, log_std
    
    def sample(self, state, epsilon=1e-6):
        mean, log_std = self.forward(state)
        std = log_std.exp()
        normal = Normal(mean, std)
        x_t = normal.rsample()
        action = torch.tanh(x_t)
        log_prob = normal.log_prob(x_t)
        log_prob -= torch.log(1 - action.pow(2) + epsilon)
        log_prob = log_prob.sum(1, keepdim=True)
        return action, log_prob, mean, log_std