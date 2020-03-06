# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
import datetime
from DQN import DQN
import tensorflow
import numpy as np
from game import Game
from monsters.stupid_monster import StupidMonster
from monsters.selfpreserving_monster import SelfPreservingMonster

# TODO This is your code!
sys.path.insert(1, '../groupNN')
from testcharacter import TestCharacter

# Hyperparameters
gamma = 0.99
copy_step = 25
num_states = 46208  # number of possble states (8*19*2*8*19) = (board dimensions for position and placing a bomb or not)
num_actions = 18  # number of possible moves (9) and bomb placment (2)
hidden_units = [200, 200]

# Exploration vs Exploitation
max_exp = 10000
min_exp = 100

batch_size = 32
lr = 1e-2

current_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
log_dir = 'logs/dqn/' + current_time
summary_writer = tensorflow.summary.create_file_writer(log_dir)

TrainNet = DQN(num_states, num_actions, hidden_units, gamma, max_exp, min_exp, batch_size, lr)
TargetNet = DQN(num_states, num_actions, hidden_units, gamma, max_exp, min_exp, batch_size, lr)

N = 500
total_rewards = np.empty(N)

epsilon = 0.99
decay = 0.9999
min_epsilon = 0.1

# Create the game
def play_game(TrainNet, TargetNet, epsilon, copy_step):

    random.seed(123) # TODO Change this if you want different random choices
    g = Game.fromfile('map.txt')
    g.add_monster(StupidMonster("stupid", # name
                                "S",      # avatar
                                3, 5,     # position
    ))
    g.add_monster(SelfPreservingMonster("aggressive", # name
                                        "A",          # avatar
                                        3, 13,        # position
                                        2             # detection range
    ))

    # TODO Add your character
    g.add_character(TestCharacter("me", # name
                                  "C",  # avatar
                                  0, 0  # position
    ))

    # Run!
    return g.go(TrainNet, TargetNet, epsilon, copy_step, 0)


for n in range(N):
    epsilon = max(min_epsilon, epsilon*decay)
    total_reward = play_game(TrainNet, TargetNet, epsilon, copy_step)
    total_rewards[n] = total_reward
    avg_rewards = total_rewards[max(0, n-100):(n+1)].mean()

    if n < N:
        print("episode:", n, "episode reward:", total_reward, "eps:", epsilon, "avg reward (last 100):", avg_rewards)
    else:
        print("avg reward for last 100 episodes:", avg_rewards)


