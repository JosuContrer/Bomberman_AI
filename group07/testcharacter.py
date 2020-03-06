# This is necessary to find the main code
import sys
import random
import numpy as np

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from queue import PriorityQueue
from sensed_world import SensedWorld
from colorama import Fore, Back
import math
from colorama import Fore, Back


class QState:
    def __init__(self, curr_pos=(-1, -1), num_enemy=-1, num_bomb=-1, dist_goal=-1, dist_enemy=-1, dist_bomb_x=False,
                 dist_bomb_y=False):
        self.curr_pos = curr_pos
        self.num_enemy = num_enemy
        self.num_bomb = num_bomb
        self.dist_goal = dist_goal
        self.dist_enemy = dist_enemy
        self.dist_bomb_x = dist_bomb_x
        self.dist_bomb_y = dist_bomb_y

    def stateToList(self):
        return [self.curr_pos, self.num_enemy, self.num_bomb, self.dist_goal, self.dist_enemy, self.dist_bomb_x,
                self.dist_bomb_y]


class QEntry:
    def __init__(self, state_elements, action_value=None):
        self.state = state_elements
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


class TestCharacter(CharacterEntity):
    colorGrid = True
    q_table = None

    def build_file(self, wrld):
        try:
            q_table = np.load('../q_table.npy', allow_pickle=True)
        except IOError:
            np.save('../q_table.npy', np.array([QEntry(self.wrldToState(wrld))], dtype=QEntry), allow_pickle=True)
            q_table = np.load('../q_table.npy', allow_pickle=True)

        return q_table

    def heuristic(self, next, goal):

        if next == goal:
            return 0

        dx = goal[0] - next[0]
        dy = goal[1] - next[1]

        h = math.sqrt(dx * dx + dy * dy)
        return round(h)

    def neighbors(self, x, y, wrld):

        n = []

        for i in range(x - 1, x + 2):
            for j in range(y - 1, y + 2):

                if (i, j) != (x, y) and i >= 0 and j >= 0 and i < wrld.width() and j < wrld.height():
                    if not wrld.wall_at(i, j):
                        # if self.colorGrid:
                        #     self.set_cell_color(x, y, Fore.CYAN)
                        n.append((i, j))

        return n

    def cost(self, current, action, wrld):
        movement = (action[0] - current[1][0], action[1] - current[1][1])
        self.qLearn(wrld)
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
                new_cost = cost_so_far[current[1]] + self.cost(current, next, wrld)
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

    def wrldToState(self, wrld):
        curr_pos = (self.x, self.y)
        num_bombs = len(wrld.bombs)
        num_enemy = len(wrld.monsters) + len(wrld.characters) - 1

        dist_goal = -1
        dist_enemy = -1
        dist_bomb_x = -1
        dist_bomb_y = -1
        if num_bombs > 0:
            for i in wrld.bombs.values():
                if curr_pos[0] == i.x:
                    dist_bomb_x = True
                    break
                if curr_pos[1] == i.y:
                    dist_bomb_y = True
                    break

        if num_enemy > 0:
            for i in wrld.monsters.values():
                temp = self.heuristic(curr_pos, (i[0].x, i[0].y))
                if temp < dist_enemy or dist_enemy == -1:
                    dist_enemy = temp
            for i in wrld.characters.values():
                if i:
                    if i[0] != self:
                        temp = self.heuristic(curr_pos, (i[0].x, i[0].y))
                        if temp < dist_enemy or dist_enemy == -1:
                            dist_enemy = temp

        curr_pos = (-1,-1)
        num_enemy = -1
        num_bombs = -1
        return QState(curr_pos, num_enemy, num_bombs, dist_goal, dist_enemy, dist_bomb_x, dist_bomb_y)

    def qLearn(self, wrld):

        # constants
        alpha = 0.8
        gamma = 0.8
        epsilon = 0
        state = self.wrldToState(wrld).stateToList()
        action = (0, 0, -1)

        if random.uniform(0, 1) > epsilon:
            for i in self.q_table:
                if i.state == state:
                    if max(i.action_value.values()) == 0:
                        key_list = []
                        for key, value in i.action_value.items():
                            if value == 0:
                                key_list.append(key)

                        action = random.choice(key_list)
                    else:
                        action = max(i.action_value, key=(lambda x: i.action_value[x]))
                        print(action, i.action_value[action])
                        break

        if action == (0, 0, -1):
            action = random.choice(list(self.q_table[0].action_value.keys()))
            print("rand", action)

        sensed_world = SensedWorld.from_world(wrld)
        next_state, next_events = sensed_world.next()
        reward = 1
        if len(next_events) > 0:
            for i in next_events:
                if i.tpe == 0 and i.character == self:
                    print('nice bomb')
                    reward += 10
                # if i.tpe == 1 and i.character == self:
                #     print('niiice bomb')
                #     reward += 50
                # if i.tpe == 2:
                #     if i.character == self and i.character != i.other:
                #         print('shmoney bomb')
                #         reward += 100
                #     elif i.character == self and i.character == i.other:
                #         print('bad bomd')
                #         reward -= 1000

        if (self.x + action[0]) >= next_state.width() or (self.y + action[1]) >= next_state.height() or (
                self.x + action[0]) < 0 or (self.y + action[1]) < 0:
            reward -= 1000
        elif next_state.wall_at((self.x + action[0]), (self.y + action[1])):
            reward -= 1000
        elif next_state.explosion_at((self.x + action[0]), (self.y + action[1])):
            reward -= 1000

        for i in self.neighbors(self.x, self.y, next_state):
            if next_state.monsters_at(i[0], i[1]):
                reward -= 100

        if self.heuristic((self.x, self.y), next_state.exitcell) > self.heuristic(
                ((self.x + action[0]), (self.y + action[1])), next_state.exitcell):
            reward += 1 / self.heuristic(((self.x + action[0]), (self.y + action[1])), next_state.exitcell)
        else:
            reward -= self.heuristic(((self.x + action[0]), (self.y + action[1])), next_state.exitcell)

        next_state = self.wrldToState(next_state).stateToList()

        old_value = 0
        for i in self.q_table:
            if i.state == state:
                old_value = i.action_value[action]
                break

        # print(old_value)
        next_max = 0
        for i in self.q_table:
            if i.state == next_state:
                next_max = max(i.action_value.values())

        print(next_max)
        new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)
        print(new_value)

        state_exists = False
        for i in range(len(self.q_table)):
            if self.q_table[i].state == state:
                state_exists = True
                new_dic = self.q_table[i].action_value
                new_dic[action] = new_value
                self.q_table[i] = QEntry(state, new_dic)
                break

        if not state_exists:
            new_entry = QEntry(state)
            new_entry.action_value[action] = new_value
            self.q_table = np.append(self.q_table, [new_entry])

        return action

    def do(self, wrld):
        # Your code here
        # start = (self.x, self.y)
        # goal = wrld.exitcell
        #
        # path = self.astar(start, goal, wrld)
        # if len(path) is not 0:
        #     dx = path[0][0] - self.x
        #     dy = path[0][1] - self.y
        #     # print("PATH: ", path)
        #     # print("CURRENT: ", self.x, self.y)
        #     # print("PATH POINTS: ", path[self.i][0], path[self.i][1])
        #     # print("MOVE: ", dx, dy)
        #     # print("GOAL: ", goal)
        # else:
        #     dx = goal[0] - self.x
        #     dy = goal[1] - self.y
        #
        # self.move(dx, dy)
        self.q_table = self.build_file(wrld)
        dx, dy, bomb = self.qLearn(wrld)
        self.move(dx, dy)
        if bomb:
            self.place_bomb()

        np.save('../q_table.npy', self.q_table, allow_pickle=True)
        for i in self.q_table:
            print(i.action_value)
