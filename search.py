grid = [[0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0]]
heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]
delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right
init = [0, 0]

goal = [len(grid) - 1, len(grid[0]) - 1]

grid_dynamic = [[0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 1, 1, 0]]

goal_dynamic = [0, 3]
cost = 1
delta_name = ['^', '<', 'v', '>']
collision_cost = 1000
success_prob = 0.5
FREE = 0

def search(grid, init, goal, cost):
    # initialize closed with zeros having same dimensions as grid
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    expand = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    action = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    init[0] = 0
    init[1] = 0
    x = init[0]
    y = init[1]
    g = 0
    closed[init[0]][init[1]] = 1
    # note that g is 0th element. because its the criteria for sorting
    open = [[g, x, y]]
    count = 0
    resign = False
    found = False
    while resign is False and found is False:
        if len(open) is 0:
            resign = True
        else:
            open.sort()
            open.reverse()
            _next = open.pop()  # remove the element with lowest g value
            g = _next[0]
            x = _next[1]
            y = _next[2]
            expand[x][y] = count
            count += 1
            if goal[0] is x and goal[1] is y:
                found = True
            else:
                for index in range(len(delta)):
                    # move forward to the next location
                    x2 = x + delta[index][0]
                    y2 = y + delta[index][1]
                    _boundary_condition = 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0])
                    _grid_is_free = closed[x2][y2] is FREE and grid[x2][y2] is FREE
                    if _boundary_condition:
                        if _grid_is_free:
                            g2 = g + cost
                            open.append([g2, x2, y2])
                            closed[x2][y2] = 1
                            """
                            saving which action from delta is performed used when tracing the policy
                            (x2,y2) are used instead of (x,y)
                            (x, y) are values of all possible tried combinations
                            (x2, y2) are values of the combination we used in the end
                            """
                            action[x2][y2] = index
    if resign is True:
        print 'Fail'
    elif found is True:
        return [g, x, y, action, expand]
    else:
        return 'Unknown Error'



