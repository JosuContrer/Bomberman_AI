# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
from game import Game

# TODO This is your code!
sys.path.insert(1, '../group07')
from testcharacter import TestCharacter


# Create the game
def run_game():
    g = Game.fromfile('map.txt')

    # TODO Add your character
    g.add_character(TestCharacter("me", # name
                                  "C",  # avatar
                                  0, 0  # position
    ))

    # Run!
    g.go(1)

for i in range(500):
    run_game()