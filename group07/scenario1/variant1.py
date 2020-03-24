# This is necessary to find the main code
import sys
import time
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
from game import Game
from monsters.selfpreserving_monster import SelfPreservingMonster

# TODO This is your code!
sys.path.insert(1, '../group07')

# Uncomment this if you want the empty test character
from testcharacter import TestCharacter

# Uncomment this if you want the interactive character
from interactivecharacter import InteractiveCharacter

# Create the game

def run_game():
    random.seed(123)
    g = Game.fromfile('map.txt')
    # g.add_monster(SelfPreservingMonster("selfpreserving", # name
    #                                     "S",              # avatar
    #                                     3, 9,             # position
    #                                     1                 # detection range
    # ))
    # TODO Add your character

    # Uncomment this if you want the test character
    g.add_character(TestCharacter("me", # name
                                  "C",  # avatar
                                  0, 0  # position
    ))

    # Uncomment this if you want the interactive character
    # g.add_character(InteractiveCharacter("me", # name
    #                                      "C",  # avatar
    #                                      0, 0  # position
    # ))

    # Run!

    # Use this if you want to press ENTER to continue at each step
    # g.go(0)

    # Use this if you want to proceed automatically
    g.go(1)


for i in range(100):
    print("Episode: ", i )
    run_game()

input()

