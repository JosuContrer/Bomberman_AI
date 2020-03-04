# This is necessary to find the main code
import sys
import random
import numpy as np
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from queue import PriorityQueue
from colorama import Fore, Back
import math
from colorama import Fore, Back


class QEntry:
    def __init__(self, state, action_value=None):
        self.state = state
        if action_value is None:
            self.action_value = {
                (-1, -1, 0): 0,
                (-1, -1, 1): 0,
                (-1, 0, 0): 0,
                (-1, 0, 1): 0,
                (-1, 1, 0): 0,
                (-1, 1, 1): 0,
                (0, -1, 0): 0,
                (0, -1, 1): 0,
                (0, 0, 0): 0,
                (0, 0, 1): 0,
                (0, 1, 0): 0,
                (0, 1, 1): 0,
                (1, -1, 0): 0,
                (1, -1, 1): 0,
                (1, 0, 0): 0,
                (1, 0, 1): 0,
                (1, 1, 0): 0,
                (1, 1, 1): 0
            }
        else:
            self.action_value = action_value


def build_file():
    try:
        q_table = np.fromfile('q_table', dtype=QEntry, count=-1)
    except:
        file = open('q_table', 'wb')
        (np.empty(0, dtype=QEntry)).tofile(file)
        q_table = np.fromfile('q_table', dtype=QEntry, count=-1)

    return q_table


class TestCharacter(CharacterEntity):

    colorGrid = True
    q_table = build_file()


    def heuristic(self, goal, next):

        if next == goal:
            return 0

        dx = goal[0] - next[0]
        dy = goal[1] - next[1]

        h = math.sqrt(dx*dx+dy*dy)
        return h

    def neighbors(self, current, wrld):

        n = []

        for x in range(current[1][0]-1, current[1][0]+2):
            for y in range(current[1][1]-1, current[1][1]+2):

                if (x, y) != current[1] and x >= 0 and y >= 0 and x < wrld.width() and y < wrld.height():
                    if not wrld.wall_at(x, y):
                        if self.colorGrid:
                            self.set_cell_color(x, y, Fore.CYAN)
                        n.append((x, y))

        return n

    def cost(self, current, next, graph):
        return 1

    def follow_path(self, start, neighbors, current, path):
        ret_val = None
        local_path = path
        for i in neighbors:
            if i == current:
                ret_val = neighbors[i]
                if ret_val is not None and ret_val is not start:
                    local_path.append(ret_val)
                    break
        if ret_val is not None:
            return self.follow_path(start, neighbors, ret_val, local_path)
        else:
            return local_path

    def astar(self, start, goal, wrld):

        graph = wrld.grid
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current[1] == goal:
                break

            for next in self.neighbors(current, wrld):
                new_cost = cost_so_far[current[1]] + self.cost(current[1], next, graph)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put((priority, next))
                    came_from[next] = current[1]

        path = self.follow_path(start, came_from, wrld.exitcell, [])
        for i in path:
            if i is not None and self.colorGrid:
                self.set_cell_color(i[0], i[1], Fore.RED)

        return [e for e in reversed(path)]

    # step = {0: [(1.0, 428, -1, False)], #
    #         1: [(1.0, 228, -1, False)],
    #         2: [(1.0, 348, -1, False)],
    #         3: [(1.0, 328, -1, False)],
    #         4: [(1.0, 328, -10, False)],
    #         5: [(1.0, 328, -10, False)],
    #         5: [(1.0, 328, -10, False)]
    #         5: [(1.0, 328, -10, False)]}

    def qLearn(self, wrld):

        #constants
        alpha = 0.1
        gamma = 0.6
        epsilon = 0.1
        state = wrld
        action = (0, 0, 0)
        next_action = (0, 0, 0)
        old_value = 0
        next_max = 0

        # For plotting metrics
        all_epochs = []
        all_penalties = []

        if random.uniform(0,1) < epsilon:
            return 1
        else:
            for i in self.q_table:
                if i.state == state:
                    action = max(i.action_value.values(), key=i.get)
                    break

        next_state = SensedWorld.from_world(wrld)
        reward = (next_state.scores[self.name] - wrld.scores[self.name])

        for i in self.q_table:
            if i.state == state:
                old_value = i.action_value[action]
            if i.state == next_state:
                next_action = max(i.action_value.values(), key=i.get)
                next_max = i.action_value[next_action]

        new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)
        state_exists = False
        for i in range(len(self.q_table)):
            if self.q_table[i].state == state:
                state_exists = True
                new_dic = self.q_table[i].action_value
                new_dic[next_action] = new_value
                self.q_table[i] = QEntry(state, new_dic)
                break

        if not state_exists:
            new_entry = QEntry(state)
            new_entry.action_value[next_action] = new_value
            self.q_table.append(new_entry)



    def do(self, wrld):
        # Your code here
        start = (self.x, self.y)
        goal = wrld.exitcell

        path = self.astar(start, goal, wrld)
        if len(path) is not 0:
            dx = path[0][0] - self.x
            dy = path[0][1] - self.y
            # print("PATH: ", path)
            # print("CURRENT: ", self.x, self.y)
            # print("PATH POINTS: ", path[self.i][0], path[self.i][1])
            # print("MOVE: ", dx, dy)
            # print("GOAL: ", goal)
        else:
            dx = goal[0] - self.x
            dy = goal[1] - self.y

        self.move(dx, dy)
