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
    colorGrid = False
    q_table = None
    show_rewards = False

    def build_file(self, wrld):
        try:
            q_table = np.load('q_table.npy', allow_pickle=True)
        except IOError:
            np.save('q_table.npy', np.array([QEntry(self.wrldToState(wrld))], dtype=QEntry), allow_pickle=True)
            q_table = np.load('q_table.npy', allow_pickle=True)

        return q_table

    def heuristic(self, next, goal):

        if next == goal:
            return 0

        dx = goal[0] - next[0]
        dy = goal[1] - next[1]

        h = math.sqrt(dx * dx + dy * dy)
        return h

    def neighbors(self, x, y, wrld, walls):

        n = []

        for i in range(x - 1, x + 2):
            for j in range(y - 1, y + 2):

                if (i, j) != (x, y) and i >= 0 and j >= 0 and i < wrld.width() and j < wrld.height():
                    if walls:
                        if not wrld.wall_at(i, j):
                        #     if self.colorGrid:
                        #         self.set_cell_color(x, y, Fore.CYAN)
                            n.append((i, j))
                    else:
                        n.append((i, j))

        return n

    def cost(self, current, action, wrld):
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


    def astar(self, start, goal, wrld, walls):

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

            for next in self.neighbors(current[1][0], current[1][1], wrld, walls):
                new_cost = cost_so_far[current[1]] + self.cost(current, next, wrld)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put((priority, next))
                    came_from[next] = current[1]

        temp = self.follow_path(start, came_from, wrld.exitcell, [])
        path = temp if temp or not walls else self.astar(start, goal, wrld, False)
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
        state_dic = {}

        # a_star = self.astar((self.x, self.y), wrld.exitcell, wrld)

        for x in range(-2, 3):
            for y in range(-2, 3):
                if self.x + x >= wrld.width() or self.x + x < 0:
                    state_dic[(x, y)] = 'e'

                elif self.y + y >= wrld.height() or self.y + y < 0:
                    state_dic[(x, y)] = 'e'

                elif wrld.wall_at((self.x + x), (self.y + y)):
                    state_dic[(x, y)] = 'w'

                elif wrld.explosion_at((self.x + x), (self.y + y)):
                    state_dic[(x, y)] = 'x'

                elif wrld.monsters_at((self.x + x), (self.y + y)):
                    state_dic[(x, y)] = 'm'

                elif wrld.characters_at((self.x + x), (self.y + y)) and (x, y) != (0, 0):
                    state_dic[(x, y)] = 'm'

                elif wrld.bomb_at((self.x + x), (self.y + y)):
                    state_dic[(x, y)] = 'b'

                elif wrld.exit_at((self.x + x), (self.y + y)):
                    state_dic[(x, y)] = 'g'

                else:
                    state_dic[(x, y)] = 'o'
        # if len(a_star):
        #     state_dic['a_star'] = (a_star[0][0] - self.x, a_star[0][1] - self.y)
        # else:
        #     state_dic['dist'] = self.y - wrld.exitcell[1]

        return state_dic

    def qLearn(self, wrld, state):

        # constants
        alpha = 0.8
        gamma = 0.8
        epsilon = 0
        action = (0, 0, -1)

        if random.uniform(0, 1) >= epsilon:
            for i in self.q_table:
                if i.state == state:
                    if max(i.action_value.values()) == 0:
                        key_list = []
                        for key, value in i.action_value.items():
                            if value == 0:
                                # if state[(key[0], key[1])] == 'a':
                                #     action = (key[0], key[1], 0)
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
        reward = 5000 / next_state.scores[self.name] * 0.3
        if len(next_events) > 0:
            for i in next_events:
                if i.tpe == 0 and i.character == self:
                    print('nice bomb')
                    reward += 10
                if i.tpe == 1 and i.character == self:
                    print('niiice bomb')
                    reward += 50
                if i.tpe == 2:
                    if i.character == self and i.character != i.other:
                        print('shmoney bomb')
                        reward += 100
                    elif i.character == self and i.character == i.other:
                        print('bad bomd')
                        reward -= 500

        if (self.x + action[0]) >= next_state.width() or (self.y + action[1]) >= next_state.height() or (
                self.x + action[0]) < 0 or (self.y + action[1]) < 0:
            reward -= 10
        elif next_state.explosion_at((self.x + action[0]), (self.y + action[1])):
            reward -= 500
        elif next_state.monsters_at((self.x + action[0]), (self.y + action[1])):
            reward -= 500
        elif next_state.wall_at((self.x + action[0]), (self.y + action[1])):
            reward -= 10

        # for i in self.neighbors((self.x + action[0]), (self.y + action[1]), next_state):
        #     if next_state.monsters_at(i[0], i[1]):
        #         reward -= 100

        if self.heuristic(
                ((self.x + action[0]), (self.y + action[1])), next_state.exitcell) == 0:
            reward += 1000

        # else:
            # print('A- Star Rewards')
            # print(action[0], state['a_star'][0])
            # print(action[1], state['a_star'][1])
            # if 'a_star' in state:
            #     if wrld.wall_at(state['a_star'][0], state['a_star'][1]):
            #         reward += action[2] * 20
            #
            #     if state['a_star'][0] == action[0]:
            #         reward += 2
            #
            #     if state['a_star'][1] == action[1]:
            #         reward += 2
            #
            # else:
            #     temp = wrld.exitcell[1] - self.y
            #     reward += (100 * 1/temp if temp > 0 else 1000) if temp < (temp + action[1]) else 0

        next_state = self.wrldToState(next_state)

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

        new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)

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

        print(reward)
        if self.show_rewards:
            file = open('rewards.csv', 'a')
            file.write('(' + str(self.x) + ' ' + str(self.y) + '),(' + str(action[0]) + ' ' + str(action[1]) + ' ' + str(action[2]) + '),' + str(reward))

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
        state = self.wrldToState(wrld)
        if ('m' in state.values()) or ('x' in state.values()):
            dx, dy, bomb = self.qLearn(wrld, state)
            self.move(dx, dy)
            if bomb:
                self.place_bomb()
            np.save('q_table.npy', self.q_table, allow_pickle=True)
        else:
            a_star = self.astar((self.x, self.y), wrld.exitcell, wrld, True)
            if a_star:
                dx, dy = a_star[0]
                dx = dx - self.x
                dy = dy - self.y
                print(self.x, self.y)
                self.move(dx, dy)
                if wrld.wall_at(self.x + dx, self.y + dy):
                    self.place_bomb()
            else:
                dx = wrld.exitcell[0] - self.x
                dy = wrld.exitcell[1] - self.y
                self.move(dx, dy)

        # for i in self.q_table:
        #     print(i.action_value)
