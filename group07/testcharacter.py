# This is necessary to find the main code
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from queue import PriorityQueue
from colorama import Fore, Back
import math
from colorama import Fore, Back

class TestCharacter(CharacterEntity):

    colorGrid = True
    i = 0

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

    def do(self, wrld):
        # Your code here
        start = (self.x, self.y)
        goal = wrld.exitcell

        path = self.astar(start, goal, wrld)
        if len(path) is not 0:
            dx = path[self.i][0] - self.x
            dy = path[self.i][1] - self.y
            # print("PATH: ", path)
            # print("CURRENT: ", self.x, self.y)
            # print("PATH POINTS: ", path[self.i][0], path[self.i][1])
            # print("MOVE: ", dx, dy)
            # print("GOAL: ", goal)
        else:
            dx = goal[0] - self.x
            dy = goal[1] - self.y

        self.move(dx, dy)
