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

            for next in self.neighbors(current,wrld):
                new_cost = cost_so_far[current[1]] + self.cost(current[1], next, graph)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put((priority, next))
                    came_from[next] = current[1]

        path = []
        for i in came_from:
            if came_from[i] is None:
                curr_key = i
            if came_from[i] is not None and self.colorGrid:
                self.set_cell_color(came_from[i][0], came_from[i][1], Fore.RED)

        print(came_from)

    def do(self, wrld):
        # Your code here
        start = (self.x, self.y)
        goal = wrld.exitcell

        self.astar((0, 0), goal, wrld)
