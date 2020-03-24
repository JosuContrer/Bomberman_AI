<p align="center"><img width=60% src="https://github.com/JosuContrer/Bomberman_AI/blob/master/group07/media/title.png"></p>

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
![Python](https://img.shields.io/badge/python-v3.6+-blue.svg)
[![Build Status](https://travis-ci.org/anfederico/Clairvoyant.svg?branch=master)](https://travis-ci.org/anfederico/Clairvoyant)
![Contributions welcome](https://img.shields.io/badge/contributions-welcome-orange.svg)

## Overview

The goal of this project was to build an AI agent using reinforcement learning to play the classic Bomberman game. The agent was capable of beating each variant in the two scenarios given. This agent was also able to blow up walls, monsters, and and tried to get to the goal quickly to accomplish a better score.

This project was done to fulfill Professors Carlo Pinciroli's Introduction to AI course offered by at Worcester Polythecnic Institute

## Setup and Installation

To run, setup, and understand this implementation of Bomberman  clone the following repo or click on this [link](https://github.com/NESTLab/CS4341-projects). The cloned folder will contain two folders `Bomberman` and `ConnectN`. Click on the `Bomberman` folder and follow the setup instructions. You can further read to have a better understanding on how this python implementations works.

```
git clone https://github.com/NESTLab/CS4341-projects.git                           
```


## Required Software
The only required software is Python 3 with the `colorama` and `pygame`
packages. To install these packages, type

```python
pip install colorama pygame                           
```

## Introduction to the Bomberman Game
Bomberman is a classic strategy maze game that consists of reaching the exit with the highest possible score. There are various implementations of this game. This implementation of Bomberman consists of two different scenarios. Each has five variants with increasing difficulties. The two scenarios have their own maze configuration, the variable that increases the difficulty is the types of monsters that are placed in each variant. There are three types of monsters: random, self-preserving, and aggressive. The aggressive monster is able to increase its aggressiveness by the cells it is allowed to detect the character. There also exists a third scenario that was not present in the code given, but it was mentioned that the classic game of Bomberman also allows for a scenario with more than one character. In this last scenario the goal of staying alive as no exit exists and to blow up the other characters before yourself.  

## Reinforcement Q-Learning Design
Q-learning is a type of model-free reinforcement learning algorithm. The decision to choose this particular algorithm was based on the need to build a learned function to solve different variants of different scenarios, and to utilize reinforcement Q-learning. There are two world scenarios, each with 5 variants for a total of 10 different world variations of Bomberman. As such, integrating a learning algorithm such as Q-learning is a preferred strategic decision since in theory, a learned solution can be obtained to solve all of the variants in a consistent manner. 

For this project the moving average Q-learning equation and A star algorithm where implemented. 
The Q-learning design was based on the article [Reinforcement Q-Learning with OpenAI Gym](https://www.learndatasci.com/tutorials/reinforcement-q-learning-scratch-python-openai-gym/) article. The following equation demonstrates the the Q-value calculations.
The A star implementation was adapted from the [Introduction to the A star Algorithm](https://www.redblobgames.com/pathfinding/a-star/introduction.html) article.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
#### Equation Key
- s: state
- a: action
- r: rewards given by world
- alpha: learning rate
- gamma: discounting factor
- max(): returns action with max Q-value

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
#### Moving Average Equation
<p align="center"><img width=50% src="https://github.com/JosuContrer/Bomberman_AI/blob/master/group07/media/moving_average.png"></p>

### Q-table
To create the Q-table used in the Bomberman algorithm, the actions were first formalized. In any given cell, the Bomberman has the option to stay still (essentially passing a turn) or move to any of the eight surrounding cells. This gives a total of nine countable actions. The Bomberman also has the ability to place a single bomb in its current cell, which can be performed along with any of the nine previous actions, which gives a total of 18 possible actions at any given cell, which is not the same as legal actions, ie. moving into a cell out of bounds or occupied by a wall, and is accounted for in the computation of the Q value. Each row entry in the table is identified by a state, which comes from the world environment, thus giving N states for any of the 18 actions. Shown below in Figure 1 is the breakdown of the different actions and how they are read in the algorithm, as well as an example Q-table.

<p align="center"><img width=75% src="https://github.com/JosuContrer/Bomberman_AI/blob/master/group07/media/qtable.png"></p>

For each possible movement (action) to the cells surrounding the bomberman’s cell, the direction can be indicated as a change in the vertical and horizontal axis, x and y respectively. A -1 represents moving to the left or up, a 1 represents moving to the right or down, and a 0 represents no motion in the x or y axis. To capture bomb placement, 1 represents the action to place a bomb and 0 to not place a bomb (default).

### World to State 
The state of the Bomberman world or environment is dependent on the location of the main character, any other character that is on the board, any monsters, any walls, any bombs or explosion cells, and the location of the exit. This is captured by the observable field or the view. However, given the size of the game board and all the cells that can be occupied by characters, monsters, bombs, and explosions, there is an obscenely large number of combinations and thus a large number of states to be accounted for. The need to account for variations in wall placement is necessary due to different world variations between the given scenarios. When constructing Q-tables the biggest challenge is to formalize the world as a state. For the first implementation of the agent the state of the Q-table saved the entire world as a state. However, this state proved to have too many variables that made learning from the state near impossible. There are issue with tracking all of these states, which primarily include length processing time of looking up and updating values on the Q-table even if it is assumed that all possible states have been recorded, as well as the constraint of file size with storing all of the states as a look-up table to be used for all 10 variants. To avoid having to store a Q-table with an infinitely large number of states in a large file, the view of the world was limited to be around the main character in the environment. Discretizing the limited view produces a state that captures only the character, monsters, walls, bombs, and explosion cells inside the view as opposed to the view of the entire game. Shown below is a summarized process of simplify state extraction from the given world.

<p align="center"><img width=75% src="https://github.com/JosuContrer/Bomberman_AI/blob/master/group07/media/world_to_state.png"></p>

As seen in the Figure, the overall view of the world is cropped to just the view around the character, which is passed into a function that discretizes the view into a state. A cell is marked for example with a ‘w’ if there is a wall, with ‘m’ if  there is a monster, and with ‘o’ if the cell is otherwise unoccupied. Increasing the size of this view will allow for taking into account more possibles and thus tracing more states. This would be beneficial if computing time and storage did not factor into the overall concern for functional performance for the project.

### World Rewards

Reward values were assigned based on events that would occur in the next state. For Bomberman, the reward of an action taken involves the presence of monsters, bombs, explosion blocks, walls, and other characters. For the purpose of creating the Q-table, arbitrary values were assigned to rewards depending on the action-state outcome associated with the reward. The general idea for assigning rewards is to have lethally bad moves to have a large negative value and winning or good moves to have large positive values. The reason for this arrangement is to both encourage “good” plays and discourage “bad” plays. In our case, “good” plays would be defined as making progress towards the exit in the map and “bad” plays would be self-destructive actions, which include blowing oneself by walking into an explosion cell (appears after a placed bomb explodes) or walking into a monster (occupying the same cell results in death).

### Results



## Authors

* **Nicholas Delli Carpini**
* **Josue Contreras**
* **Justin Cheng**

## Acknowledgments

* Reinforcement Q-Learning with OpenAI Gym [article](https://www.learndatasci.com/tutorials/reinforcement-q-learning-scratch-python-openai-gym/)