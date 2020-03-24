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
from events import Event


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
    q_table = None

    debug_print = True
    show_rewards = False
    colorGrid = False

    train = True
    add_monster_move = True

    moving_alpha = False
    alpha = 0.8
    gamma = 0.8
    epsilon = 0.0

    give_rewards = [
        'life_penalty',
        'wall_blow',
        'mon_blow',
        'char_blow',
        'self_blow',
        'mon_die',
        'illegal_move',
        'explosion_move',
        'a_star_move',
        'a_star_bomb',
        'mon_move',
        'exit'
    ]

    view_cats = [
        'invalid',
        'wall',
        'explosion',
        'monster',
        'character',
        'bomb',
        'exit',
        'none',
        'a_star',
        'used_bomb'
    ]

    def build_file(self, wrld):
        try:
            q_table = np.load('brain.npy', allow_pickle=True)
        except IOError:
            np.save('brain.npy', np.array([QEntry(self.wrld_to_state(wrld))], dtype=QEntry), allow_pickle=True)
            q_table = np.load('brain.npy', allow_pickle=True)

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

        return path

    def wrld_to_state(self, wrld):
        state_dic = {}

        a_star = self.astar((self.x, self.y), wrld.exitcell, wrld, True)

        for x in range(-3, 4):
            for y in range(-3, 4):
                if 'invalid' in self.view_cats and self.x + x >= wrld.width() or self.x + x < 0:
                    state_dic[(x, y)] = 'e'

                elif 'invalid' in self.view_cats and self.y + y >= wrld.height() or self.y + y < 0:
                    state_dic[(x, y)] = 'e'

                elif wrld.wall_at((self.x + x), (self.y + y)) and 'wall' in self.view_cats:
                    state_dic[(x, y)] = 'w'

                elif wrld.explosion_at((self.x + x), (self.y + y)) and 'explosion' in self.view_cats:
                    state_dic[(x, y)] = 'x'

                elif wrld.monsters_at((self.x + x), (self.y + y)) and 'monster' in self.view_cats:
                    state_dic[(x, y)] = 'm'

                # elif wrld.characters_at((self.x + x), (self.y + y)) and (x, y) != (0, 0) and 'character' in self.view_cats:
                #     state_dic[(x, y)] = 'm'

                elif wrld.bomb_at((self.x + x), (self.y + y)) and 'bomb' in self.view_cats:
                    state_dic[(x, y)] = 'b'

                elif wrld.exit_at((self.x + x), (self.y + y)) and 'exit' in self.view_cats:
                    state_dic[(x, y)] = 'g'

                else:
                    if 'none' in self.view_cats:
                        state_dic[(x, y)] = 'o'
        if len(a_star):
            state_dic['a_star'] = (a_star[-1][0] - self.x, a_star[-1][1] - self.y)
        else:
            state_dic['a_star'] = (wrld.exitcell[0] - self.x, wrld.exitcell[1] - self.y)

        state_dic['bomb'] = True if 'used_bomb' in self.view_cats and wrld.bombs else False

        return state_dic

    def qLearn(self, wrld, state):

        # constants
        alpha = self.alpha
        gamma = self.gamma
        epsilon = self.epsilon
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
                        if self.debug_print:
                            print(action, i.action_value[action])

                    else:
                        action = max(i.action_value, key=(lambda x: i.action_value[x]))
                        if self.debug_print:
                            print(action, i.action_value[action])
                        break

        if self.train:
            if action == (0, 0, -1):
                action = random.choice(list(self.q_table[0].action_value.keys()))
                if self.debug_print:
                    print("rand", action)

            action_backup = action

            if 'illegal_move' in self.give_rewards:
                if ((self.x + action[0]) >= wrld.width() or (self.x + action[0]) < 0) and (
                        (self.y + action[1]) >= wrld.height() or (self.y + action[1]) < 0):
                    if self.debug_print:
                        print('illegal_move')
                    action = random.choice(list(self.q_table[0].action_value.keys()))
                elif (self.x + action[0]) >= wrld.width() or (self.x + action[0]) < 0:
                    if self.debug_print:
                        print('illegal_move')
                    action = (0, action[1], action[2])
                elif (self.y + action[1]) >= wrld.height() or (self.y + action[1]) < 0:
                    if self.debug_print:
                        print('illegal_move')
                    action = (action[0], 0, action[2])
                elif wrld.wall_at((self.x + action[0]), (self.y + action[1])):
                    if self.debug_print:
                        print('wall_move')
                    action = random.choice(list(self.q_table[0].action_value.keys()))
                elif wrld.wall_at((self.x + action[0]), (self.y)):
                    if self.debug_print:
                        print('wall_move')
                    action = (0, action[1], action[2])
                elif wrld.wall_at((self.x), (self.y + action[1])):
                    if self.debug_print:
                        print('wall_move')
                    action = (action[0], 0, action[2])

            sensed_world = SensedWorld.from_world(wrld)
            (sensed_world.me(self)).move(action[0], action[1])
            if action[2]:
                (sensed_world.me(self)).place_bomb()
            next_state, next_events = sensed_world.next()

            reward = (5000 / wrld.scores[self.name] * 0.5) if 'life_penalty' in self.give_rewards else 0

            if len(next_events) > 0:
                for i in next_events:
                    if i.tpe == Event.CHARACTER_FOUND_EXIT:
                        print("Found Exit!!!!!!!!!!!!!!!!!")
                    if 'wall_blow' in self.give_rewards and i.tpe == 0 and i.character.name == self.name:
                        if self.debug_print:
                            print('wall_blow')
                        reward += 10
                    if 'mon_blow' in self.give_rewards and i.tpe == 1 and i.character.name == self.name:
                        if self.debug_print:
                            print('mon_blow')
                        reward += 50
                    if i.tpe == 2:
                        if 'char_blow' in self.give_rewards and i.character.name == self.name and i.character.name != i.other.name:
                            if self.debug_print:
                                print('char_blow')
                            reward += 100
                        elif 'self_blow' in self.give_rewards and i.character.name == self.name and i.character.name == i.other.name:
                            if self.debug_print:
                                print('self_blow')
                            reward -= 500
                    if 'mon_die' in self.give_rewards and i.tpe == 3 and i.character.name == self.name:
                        if self.debug_print:
                            print('mon_die')
                        reward -= 500

            elif 'explosion_move' in self.give_rewards and (next_state.explosion_at((self.x + action[0]), (self.y + action[1])) or wrld.explosion_at((self.x + action[0]), (self.y + action[1]))):
                if self.debug_print:
                    print('explosion_move')
                reward -= 500

            if 'exit' in self.give_rewards and self.heuristic(
                    ((self.x + action[0]), (self.y + action[1])), next_state.exitcell) == 0:
                if self.debug_print:
                    print('exit')
                reward += 10000

            elif 'a_star_move' in self.give_rewards:
                temp_for_print = reward
                if 'a_star' in state:
                    if 'a_star_bomb' in self.give_rewards and wrld.wall_at(self.x + state['a_star'][0], self.y + state['a_star'][1]) and not state['bomb']:
                        if self.debug_print:
                            print('a_star_bomb')
                        reward += action[2] * 20

                    if (action[0], action[1]) != (0, 0):
                        if (state['a_star'][0] == action[0]) and (state['a_star'][1] == action[1]):
                            reward += 3

                        elif (state['a_star'][0] == action[0]) and (abs(state['a_star'][1] - action[1]) < 2):
                            reward += 1

                        elif (state['a_star'][1] == action[1]) and (abs(state['a_star'][0] - action[0]) < 2):
                            reward += 1

                        else:
                            reward -= 1

                    if self.debug_print:
                        print('a_star (%d, %d) -> %d' % (state['a_star'][0], state['a_star'][1], reward - temp_for_print))

            # Current State
            curr_state = state

            # Next state is created here
            next_state = self.wrld_to_state(next_state)

            if self.add_monster_move:
                ######## States are compared ##########
                # Monster position tracking
                (mon_curr_x, mon_curr_y) = (0,0)
                (mon_next_x, mon_next_y) = (0,0)
                offset_x = 4
                offset_y = 4

                # Current State
                curr_state_list = []
                for y in range(-3, 4):
                    row = []
                    for x in range(-3, 4):
                        state_item = state[(x,y)]
                        row.append(state[(x, y)])

                        if state_item == 'm':
                            (mon_curr_x, mon_curr_y) = (x+offset_x,y+offset_y)
                    curr_state_list.append(row)

                # Next State
                next_state_list = []
                for y in range(-3, 4):
                    row = []
                    for x in range(-3, 4):
                        state_item = next_state[(x,y)]
                        row.append(next_state[(x, y)])
                        if state_item == 'm':
                            (mon_next_x, mon_next_y) = (x+offset_x,y+offset_y)
                    next_state_list.append(row)

                print("current monster:", mon_curr_x , mon_curr_y)
                print("next monster:", mon_next_x, mon_next_y)

                # Check monster movement towards character
                current_monster_to_character = self.heuristic((offset_x, offset_y), (mon_curr_x, mon_curr_y))
                next_monster_to_character = self.heuristic((offset_x, offset_y), (mon_next_x, mon_next_y))

                # Check if monster moved in either axis
                (mon_dx,mon_dy) = (mon_curr_x - mon_next_x, mon_curr_y - mon_next_y)
                if abs(mon_dx) == 1 or abs(mon_dy) == 1:
                    if self.debug_print:
                        print("MONSTER MOVED")
                    if next_monster_to_character < current_monster_to_character:
                        if self.debug_print:
                            print("MONSTER CLOSER TO ME")
                        reward -= 5

                print("monster move: ", mon_dx, mon_dy)
                print("current mc: ", current_monster_to_character, "next mc: ", next_monster_to_character)
                print("next action: ", action[0], action[1])


                if self.debug_print:
                    print("Current State Character View Map")
                    for row in curr_state_list:
                        print(row)

                    print("Next State Character View Map")
                    for row in next_state_list:
                        print(row)



            # Manage rewards when a monster is seen
            if 'mon_move' in self.give_rewards:
                if 'm' in next_state.values():
                    for key, value in next_state.items():
                        if value == 'm':
                            mx = 1 if key[0] > 0 else (-1 if key[0] < 0 else 0)
                            my = 1 if key[1] > 0 else (-1 if key[1] < 0 else 0)

                            if (action[0], action[1]) != (0, 0):
                                if (mx == -action[0]) and (my == -action[1]):
                                    reward += 3

                                elif (mx == -action[0]) and (abs(my - action[1]) < 2):
                                    reward += 1

                                elif (my == -action[1]) and (abs(mx - action[0]) < 2):
                                    reward += 1

            old_value = 0
            old_value_backup = 0
            for i in self.q_table:
                if i.state == state:
                    old_value = i.action_value[action]
                    old_value_backup = i.action_value[action_backup]
                    break

            next_max = 0
            for i in self.q_table:
                if i.state == next_state:
                    next_max = max(i.action_value.values())

            new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)
            new_value_backup = (1 - alpha) * old_value_backup + alpha * (reward + gamma * next_max)

            state_exists = False
            for i in range(self.q_table.size):
                if self.q_table[i].state == state:
                    state_exists = True
                    new_dic = self.q_table[i].action_value
                    new_dic[action] = new_value
                    if action != action_backup:
                        new_dic[action_backup] = new_value_backup
                    self.q_table[i] = QEntry(state, new_dic)
                    break

            if not state_exists:
                new_entry = QEntry(state)
                new_entry.action_value[action] = new_value
                if action != action_backup:
                    new_entry.action_value[action_backup] = new_value_backup
                self.q_table = np.append(self.q_table, [new_entry])

            if self.show_rewards:
                file = open('rewards.csv', 'a')
                file.write('(' + str(self.x) + ' ' + str(self.y) + '),(' + str(action[0]) + ' ' + str(action[1]) + ' ' + str(action[2]) + '),' + str(reward) + ',\n')

        return action

    def do(self, wrld):
        # Your code here
        self.q_table = self.build_file(wrld)
        state = self.wrld_to_state(wrld)

        dx, dy, bomb = self.qLearn(wrld, state)
        self.move(dx, dy)
        if bomb:
            self.place_bomb()

        np.save('brain.npy', self.q_table, allow_pickle=True)