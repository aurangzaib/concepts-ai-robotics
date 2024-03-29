from class_robot import Robot
from numpy import *


class plan:
    # --------
    # init:
    #    creates an empty plan
    #

    def __init__(self, grid, init, goal, cost=1):
        self.cost = cost
        self.grid = grid
        self.init = init
        self.goal = goal
        self.make_heuristic(grid, goal, self.cost)
        self.path = []
        self.s_path = []

    # --------
    #
    # make heuristic function for a grid

    def make_heuristic(self, grid, goal, cost):
        self.heuristic = [[0 for row in range(len(grid[0]))]
                          for col in range(len(grid))]
        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                self.heuristic[i][j] = abs(i - self.goal[0]) + \
                                       abs(j - self.goal[1])

    # ------------------------------------------------
    #
    # A* for searching a path to the goal
    #
    #

    def astar(self, goal):

        if self.heuristic == []:
            raise ValueError, "Heuristic must be defined to run A*"

        # internal motion parameters
        delta = [[-1, 0],  # go up
                 [0, -1],  # go left
                 [1, 0],  # go down
                 [0, 1]]  # do right

        # open list elements are of the type: [f, g, h, x, y]

        # initialize with same dimensions as grid
        closed = [[0 for row in range(len(self.grid[0]))] for col in range(len(self.grid))]
        expand = [[-1 for row in range(len(self.grid[0]))] for col in range(len(self.grid))]
        action = [[0 for row in range(len(self.grid[0]))] for col in range(len(self.grid))]

        closed[self.init[0]][self.init[1]] = 1

        x = self.init[0]
        y = self.init[1]
        h = self.heuristic[x][y]
        g = 0
        f = g + h
        closed[self.init[0]][self.init[1]] = 1
        open = [[f, g, h, x, y]]

        found = False  # flag that is set when search complete
        resign = False  # flag set if we can't find expand
        count = 0

        while not found and not resign:

            # check if we still have elements on the open list
            if len(open) == 0:
                resign = True
                print '###### Search terminated without success'
            else:
                # remove node from list
                open.sort()
                open.reverse()
                next = open.pop()
                x = next[3]
                y = next[4]
                g = next[1]

                f = next[0]
                g = next[1]
                h = next[2]
                x = next[3]
                y = next[4]
                expand[x][y] = count
            # check if we are done

            if x == goal[0] and y == goal[1]:
                found = True
                # print '###### A* search successful'

            else:
                # expand winning element and add to new open list
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if 0 <= x2 < len(self.grid) and 0 <= y2 < len(self.grid[0]):
                        if closed[x2][y2] == 0 and self.grid[x2][y2] == 0:
                            g2 = g + self.cost
                            h2 = self.heuristic[x2][y2]
                            f2 = g2 + h2
                            open.append([f2, g2, h2, x2, y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i

            count += 1

        # extract the path



        invpath = []
        x = self.goal[0]
        y = self.goal[1]
        invpath.append([x, y])
        while x != self.init[0] or y != self.init[1]:
            x2 = x - delta[action[x][y]][0]
            y2 = y - delta[action[x][y]][1]
            x = x2
            y = y2
            invpath.append([x, y])

        self.path = []
        for i in range(len(invpath)):
            self.path.append(invpath[len(invpath) - 1 - i])

    # ------------------------------------------------
    #
    # this is the smoothing function
    #




    def smooth(self, weight_data=0.1, weight_smooth=0.1,
               tolerance=0.000001):

        if self.path == []:
            raise ValueError, "Run A* first before smoothing path"

        self.s_path = [[0 for row in range(len(self.path[0]))] for col in range(len(self.path))]
        for i in range(len(self.path)):
            for j in range(len(self.path[0])):
                self.s_path[i][j] = self.path[i][j]

        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(self.path) - 1):
                for j in range(len(self.path[0])):
                    aux = self.s_path[i][j]

                    self.s_path[i][j] += weight_data * (self.path[i][j] - self.s_path[i][j])

                    self.s_path[i][j] += weight_smooth * \
                                         (self.s_path[i - 1][j] + self.s_path[i + 1][j]
                                          - (2.0 * self.s_path[i][j]))
                    if i >= 2:
                        self.s_path[i][j] += 0.5 * weight_smooth * \
                                             (2.0 * self.s_path[i - 1][j] - self.s_path[i - 2][j]
                                              - self.s_path[i][j])
                    if i <= len(self.path) - 3:
                        self.s_path[i][j] += 0.5 * weight_smooth * \
                                             (2.0 * self.s_path[i + 1][j] - self.s_path[i + 2][j]
                                              - self.s_path[i][j])

            change += abs(aux - self.s_path[i][j])
