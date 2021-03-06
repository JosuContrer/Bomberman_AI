<p align="center"><img width=60% src="https://github.com/JosuContrer/Bomberman_AI/blob/master/group07/media/title.png"></p>

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
![Python](https://img.shields.io/badge/python-v3-blue.svg)
![Contributions welcome](https://img.shields.io/badge/contributions-welcome-orange.svg)

## Overview

The goal of this project was to build an AI agent using reinforcement learning to play the classic Bomberman game. The agent was capable
 of beating each variant in the two scenarios given. This agent was also able to blow up walls, monsters, and and tried to get to the 
 goal quickly to accomplish a better score.

This project was done to fulfill Professors Carlo Pinciroli's Introduction to AI course offered by at Worcester Polythecnic Institute

## Required Software
The only required software is Python 3 with the `colorama` and `pygame`
packages. To install these packages, type

```python
pip install colorama pygame                           
```

## Setup and Installation

To run, setup, and understand this implementation of Bomberman  clone the following repo or click on this [link](https://github.com/NESTLab/CS4341-projects).
 The cloned folder will contain two folders `Bomberman` and `ConnectN`. Click on the `Bomberman` folder and follow the setup instructions. You can further 
 read to have a better understanding on how this python implementations works.

```
git clone https://github.com/NESTLab/CS4341-projects.git                           
```

After that clone this repo or download the zip and substitute the `groupNN` folder with the folder in this repo you downloaded. After that you are ready to 
run this project.

## Introduction to the Bomberman Game
Bomberman is a classic strategy maze game that consists of reaching the exit with the highest possible score. There are various implementations of this game. 
This implementation of Bomberman consists of two different scenarios. Each has five variants with increasing difficulties. The two scenarios have their own 
maze configuration, the variable that increases the difficulty is the types of monsters that are placed in each variant. There are three types of monsters: 
random, self-preserving, and aggressive. The aggressive monster is able to increase its aggressiveness by the cells it is allowed to detect the character. 
There also exists a third scenario that was not present in the code given, but it was mentioned that the classic game of Bomberman also allows for a scenario
 with more than one character. In this last scenario the goal of staying alive as no exit exists and to blow up the other characters before yourself.  

## Reinforcement Q-Learning Design
Q-learning is a type of model-free reinforcement learning algorithm. The decision to choose this particular algorithm was based on the need to build a learned
 function to solve different variants of different scenarios, and to utilize reinforcement Q-learning. There are two world scenarios, each with 5 variants for
  a total of 10 different world variations of Bomberman. As such, integrating a learning algorithm such as Q-learning is a preferred strategic decision since 
  in theory, a learned solution can be obtained to solve all of the variants in a consistent manner. 

For this project the moving average Q-learning equation and A star algorithm where implemented. 
The Q-learning design was based on the article [Reinforcement Q-Learning with OpenAI Gym](https://www.learndatasci.com/tutorials/reinforcement-q-learning-scratch-python-openai-gym/) article.
 The following equation demonstrates the the Q-value calculations.The A star implementation was adapted from the [Introduction to the A star Algorithm](https://www.redblobgames.com/pathfinding/a-star/introduction.html) article.

#### Equation Key
- s: state
- a: action
- r: rewards given by world
- alpha: learning rate
- gamma: discounting factor
- max(): returns action with max Q-value

#### Moving Average Equation
<p align="center"><img width=50% src="https://github.com/JosuContrer/Bomberman_AI/blob/master/group07/media/moving_average.png"></p>

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
### Q-table
To create the Q-table used in the Bomberman algorithm, the actions were first formalized. In any given cell, the Bomberman has the option to stay 
still (essentially passing a turn) or move to any of the eight surrounding cells. This gives a total of nine countable actions. The Bomberman also 
has the ability to place a single bomb in its current cell, which can be performed along with any of the nine previous actions, which gives a total 
of 18 possible actions at any given cell, which is not the same as legal actions, ie. moving into a cell out of bounds or occupied by a wall, and is 
accounted for in the computation of the Q value. Each row entry in the table is identified by a state, which comes from the world environment, thus 
giving N states for any of the 18 actions. Shown below in Figure 1 is the breakdown of the different actions and how they are read in the algorithm, 
as well as an example Q-table.

<p align="center"><img width=58% src="https://github.com/JosuContrer/Bomberman_AI/blob/master/group07/media/qtable.png"></p>

For each possible movement (action) to the cells surrounding the bomberman’s cell, the direction can be indicated as a change in the vertical and 
horizontal axis, x and y respectively. A -1 represents moving to the left or up, a 1 represents moving to the right or down, and a 0 represents no 
motion in the x or y axis. To capture bomb placement, 1 represents the action to place a bomb and 0 to not place a bomb (default).

### World to State 
The state of the Bomberman world or environment is dependent on the location of the main character, any other character that is on the board, any 
monsters, any walls, any bombs or explosion cells, and the location of the exit. This is captured by the observable field or the view. However, given 
the size of the game board and all the cells that can be occupied by characters, monsters, bombs, and explosions, there is an obscenely large number of 
combinations and thus a large number of states to be accounted for. The need to account for variations in wall placement is necessary due to different 
world variations between the given scenarios. When constructing Q-tables the biggest challenge is to formalize the world as a state. For the first 
implementation of the agent the state of the Q-table saved the entire world as a state. However, this state proved to have too many variables that made 
learning from the state near impossible. There are issue with tracking all of these states, which primarily include length processing time of looking up 
and updating values on the Q-table even if it is assumed that all possible states have been recorded, as well as the constraint of file size with storing 
all of the states as a look-up table to be used for all 10 variants. To avoid having to store a Q-table with an infinitely large number of states in a large 
file, the view of the world was limited to be around the main character in the environment. Discretizing the limited view produces a state that captures only 
the character, monsters, walls, bombs, and explosion cells inside the view as opposed to the view of the entire game. Shown below is a summarized process of 
simplify state extraction from the given world.

<p align="center"><img width=58% src="https://github.com/JosuContrer/Bomberman_AI/blob/master/group07/media/world_to_state.png"></p>

As seen in the Figure, the overall view of the world is cropped to just the view around the character, which is passed into a function that discretizes the 
view into a state. A cell is marked for example with a ‘w’ if there is a wall, with ‘m’ if  there is a monster, and with ‘o’ if the cell is otherwise unoccupied. 
Increasing the size of this view will allow for taking into account more possibles and thus tracing more states. This would be beneficial if computing time and 
storage did not factor into the overall concern for functional performance for the project.

### World Rewards

Reward values were assigned based on events that would occur in the next state. For Bomberman, the reward of an action taken involves the presence of monsters, 
bombs, explosion blocks, walls, and other characters. For the purpose of creating the Q-table, arbitrary values were assigned to rewards depending on the 
action-state outcome associated with the reward. The general idea for assigning rewards is to have lethally bad moves to have a large negative value and winning 
or good moves to have large positive values. The reason for this arrangement is to both encourage “good” plays and discourage “bad” plays. In our case, “good” 
plays would be defined as making progress towards the exit in the map and “bad” plays would be self-destructive actions, which include blowing oneself by walking 
into an explosion cell (appears after a placed bomb explodes) or walking into a monster (occupying the same cell results in death).

## Results
### Scenario 1

The following animations demonstrate the trained Agent exiting successfully the maze while taking the fastest path and avoiding the monsters. Variant 2 below has 
two animations (demo 1 and 2) to show two different Q-tables trained on the same Variant and the two different solutions learnt by the Agent. Variant 5 shows the 
Agent beating the hardest level in this scenario. **Note:** the video for Variant 5 below was taken with a slower frame rate and the Agent was running a lot faster. This resulted is a video 
where the Agent seems to jump around, but this is not the case as it was able to move as the other two animations.  


Variant 2 Demo 1           |  Variant 2 Demo 2         |  Variant 5 Demo
:-------------------------:|:-------------------------:|:-------------------------:
-<p><img width=50% src="https://github.com/JosuContrer/Bomberman_AI/blob/master/group07/media/scenario1_variant2_sdemo.gif"></p>- | -<p><img width=50% src="https://github.com/JosuContrer/Bomberman_AI/blob/master/group07/media/scenario1_variant2_sdemo2.gif"></p>- | -<p><img width=50% src="https://github.com/JosuContrer/Bomberman_AI/blob/master/group07/media/scenario1_variant5_sdemo.gif"></p>-

* **Episode:** is a representation of a complete game played, for bomberman a game starts with the character at position (0,0)  and ends if the character reaches 
the exit, is killed by a bomb, killed by a monster, or max time is reached.

The plot below shows the averaged reward value for every 3 episodes, a total of 136 episodes. After episode 49 the agent is able to learn the best Q-values, 
therefore giving it the best action based on the current state. Before episode 49 the rewards were negative. They seem to fluctuate around the -25 starting value. 
This behavior is interesting since the rewards at first start to increase and then decrease to below the -25 reward value. After that they start to reach the -25 value 
and once they do they immediately jump to the final positive reward of 50. A complete period allows for the agent Scenario 1 Variant 5 to learn from the environment the 
best Q-values for the states and determine the best action to take. After the best policy is determined, the agent is able to reach the exit 90 % of the time.

<p align="center"><img width=58% src="https://github.com/JosuContrer/Bomberman_AI/blob/master/group07/media/scenario1_variant5_rewards_table.png"></p>

#### Scenario 2

The following animations show the trained Agent learning in Variant 5 and beating the game in Variant 3. The Agent was successful at getting to the exit till Variant 4, 
but it was only able to beat Variant 5 one time. 

Variant 5 Learning         |  Variant 3 Trained         
:-------------------------:|:-------------------------:
-<p><img width=32% src="https://github.com/JosuContrer/Bomberman_AI/blob/master/group07/media/scenario2_variant2_learning_demo.gif"></p>- | -<p><img width=50% src="https://github.com/JosuContrer/Bomberman_AI/blob/master/group07/media/scenario2_variant3_sdemo.gif"></p>-

The plot below demonstrates an Agent that starts with an unpopulated Q-table (no states, no actions, and no Q-values) in Scenario 2 Variant 1-4. The hyperparameters 
were set to the following: learning rate of 0.8, discount factor of 0.8, and epsilon of 0. Rewards were unchanged as they are constants given by the environment. Figure 5 
below shows a moving average of the reward value  for a total of 1,300 episodes. After episode 573 the agent starts to learn the Q-values that give the higher rewards. 
After episode 959 the rewards begin to stabilize around 90.

<p align="center"><img width=58% src="https://github.com/JosuContrer/Bomberman_AI/blob/master/group07/media/scenario2_variant4_rewards_table.png"></p>

* **Important Note:** When training an Agent remember to save the "brain" (Q-table) when it is successfull. This helps when trying recreate the the Agent beating each
 variant in both scenarios. It also allows you to screen record it to be able to show it in reports and github readme's like this one. That is why Variants 4 and 5 of 
 Scenario 1 are not showed in the animations above.

## Authors

* **Nicholas Delli Carpini**
* **Josue Contreras**
* **Justin Cheng**

## Acknowledgments

* Reinforcement Q-Learning with OpenAI Gym [article](https://www.learndatasci.com/tutorials/reinforcement-q-learning-scratch-python-openai-gym/)
* Introduction to the A star Algorithm [article](https://www.redblobgames.com/pathfinding/a-star/introduction.html)
