from random import choice
from random import randint

grid = [[0, 0, 1, 0, 1, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 1, 0],
        [0, 0, 1, 1, 1, 0]]
# manhattan distance heuristic
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

grid_dynamic = [[0, 0, 0, 1, 0, 0, 0],
                [0, 1, 0, 0, 0, 1, 0],
                [0, 1, 1, 0, 1, 1, 0],
                [0, 1, 1, 1, 1, 1, 0],
                [0, 0, 0, 0, 0, 0, 0]]

goal_dynamic = [0, 6]
cost = 1
delta_name = ['^', '<', 'v', '>']
collision_cost = 100
success_prob = 0.8
FREE = 0


# Breadth First Planning
def breadth_first_search(_grid, _delta, _init, _goal, _cost):
    # initialize closed with zeros having same dimensions as grid
    _closed = [[0 for col in range(len(_grid[0]))] for row in range(len(_grid))]
    _expand = [[-1 for col in range(len(_grid[0]))] for row in range(len(_grid))]
    _action = [[-1 for col in range(len(_grid[0]))] for row in range(len(_grid))]
    _init[0], _init[1] = 0, 0
    g, row, col = 0, _init[0], _init[1]
    _closed[_init[0]][_init[1]] = 1
    # note that g is 0th element. g is the criteria for sorting
    _open = [[g, row, col]]
    count = 0
    resign = False
    found = False
    while resign is False and found is False:
        if len(_open) is 0:
            resign = True
        else:
            # remove the element with lowest g value
            _open.sort()
            _open.reverse()
            _next = _open.pop()

            g, row, col = _next[0], _next[1], _next[2]
            _expand[row][col] = count
            count += 1

            if _goal[0] is row and _goal[1] is col:
                found = True
            else:
                for index in range(len(_delta)):
                    """
                    move forward to the next location
                    """
                    new_row = row + _delta[index][0]
                    new_col = col + _delta[index][1]
                    _boundary_condition = 0 <= new_row < len(_grid) and 0 <= new_col < len(_grid[0])
                    if _boundary_condition:
                        _grid_is_free = _closed[new_row][new_col] is FREE and _grid[new_row][new_col] is FREE
                        if _grid_is_free:
                            g2 = g + _cost
                            _open.append([g2, new_row, new_col])
                            _closed[new_row][new_col] = 1
                            """
                            saving which action from delta is performed used when tracing the policy
                            (x2, y2) are used instead of (x, y)
                            (x, y) are values of all possible tried combinations
                            (x2, y2) are values of the combination we used in the end
                            """
                            _action[new_row][new_col] = index
    if resign is True:
        print('Fail')
    elif found is True:
        return [g, row, col, _action, _expand]
    else:
        return 'Unknown Error'


# A* Planning
def heuristic_search(grid, heuristic, init, goal, cost):
    """
    A* method can reduce number of expanded nodes by calculating the new f value
    f = g + h(x,y)
    so whenever it has choice between 2 options it will select one with smaller f value but A* method is not
    efficient when both options have same f values. try using 2nd col as: [0 1 1 1 1 1] instead of [0 1 1 1 1 1]
    and A* won't work very well as there will be several options with same f values.

    run with:
    grid = [[0, 0, 0, 0, 0, 0], [0, 1, 1, 1, 1, 0], [0, 1, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]
    and see how A* and breadth-first methods work
    breadth-first method will explore a lot of places hence high value of expand.

    A* is very good in real world where we have dead end obstacles.
    this grid based A* is different for real world cars as cars can turn at an angle.
    """
    # initialize with same dimensions as grid
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

    init[0], init[1] = 0, 0
    g, row, col = 0, init[0], init[1]
    h = heuristic[row][col]
    f = g + h

    # note that f is 0th element. f is the criteria for sorting
    closed[init[0]][init[1]] = 1
    open = [[f, g, row, col]]
    count = 0
    resign = False
    found = False
    while not resign and not found:
        if len(open) is 0:
            resign = True
        else:
            open.sort()
            open.reverse()
            _next = open.pop()  # remove the element with lowest f value

            f, g, row, col = _next[0], _next[1], _next[2], _next[3]
            expand[row][col] = count
            count += 1

            if goal[0] is row and goal[1] is col:
                found = True
            else:
                for index in range(len(delta)):
                    """
                    move forward to the next location
                    and get x,y of the expanded node
                    """
                    x2 = row + delta[index][0]
                    y2 = col + delta[index][1]
                    if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]):
                        if closed[x2][y2] is FREE and grid[x2][y2] is FREE:
                            g2 = g + cost
                            f2 = g2 + heuristic[x2][y2]
                            open.append([f2, g2, x2, y2])
                            closed[x2][y2] = 1
                            """
                            saving which action from delta is performed used when tracing the policy (path)
                            (x2,y2) are used instead of (x,y)
                            (x, y) are values of all possible tried combinations
                            (x2, y2) are values of the combination we used in the end
                            """
                            action[x2][y2] = index
    if resign is True:
        print('Fail')
    elif found is True:
        return [f, g, row, col, action, expand]
    else:
        return 'Unknown Error'


# Trace the path created by Breadth First Planning
def trace_search(grid, init, goal, _action):
    """
    mark the goal coordinates as *

    we took next positions when using search algorithm:
        x2 = x + delta(index)(0); y2 = y + delta(index)(1)

    here we will go in reverse direction and perform the recursion:
        x2 = x - delta(action(x)(y))(0); y2 = y - delta(action(x)(y))(1)

    where x2,y2 are original coord.; x,y are current coord.

    then we draw the character based on which action was performed
    action[x][y] is the index from delta and every delta element has its assoc. character in the delta_name vector

    then save updated x, y with x2, y2 i.e. recursion
    """
    policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    row, col = goal[0], goal[1]
    policy[row][col] = '*'
    while row != init[0] or col != init[1]:
        new_row = row - delta[_action[row][col]][0]
        new_col = col - delta[_action[row][col]][1]
        policy[new_row][new_col] = delta_name[_action[row][col]]
        row, col = new_row, new_col
    for p in policy:
        print(p)
    return policy


# Optimal Policy on each Node or Location
def dynamic_policy(_grid, _goal, _cost, _delta):
    # initialize value with 99
    _value = [[99 for row in range(len(_grid[0]))] for col in range(len(_grid))]
    _policy = [[' ' for row in range(len(_grid[0]))] for col in range(len(_grid))]
    _change = True
    """
    for x in rows
        for y in cols
            if x,y match with goals then mark value(x,y) as 0
            for k in delta
                x2 -> x + delta(k)(0) and y2 -> y + delta(k)(0)
                check:
                1- x2, y2 are within grid
                2- grid(x2)(y2) is free i.e. grid(x2)(y2) is 0
                    get value of x2,y2 and add cost to it
                    i.e. v2 = value(x2)(y2) + cost
                    at this point value(x2)(y2) gives new pose values and value(x)(y) gives original pose values
                    -> see below diagram for delta loop <-
                    compare v2 and v(x)(y)
                    if v2 is smaller which initially would only be for goal location
                        then original location value = new value
                        i.e. value(x2)(y2) = v2
                        so:
                        |   |                                   |   |
                      __________                             __________
                        |   | 99             --->               |   | 1
                      __________                             __________
                        |   | 0                                 |   | 0
    """
    while _change is True:
        _change = False
        # loop over each position in the grid
        # vertical loop
        for row in range(len(_grid)):
            # horizontal loop
            for col in range(len(_grid[0])):
                # use debugger at this point to see how its proceeding
                """
                if we have reached on the goal position and value is larger. then assign it 0.
                goal in dynamic programming is starting point
                """
                if row is _goal[0] and col is _goal[1]:
                    if _value[row][col] > 0:
                        _value[row][col] = 0
                        _policy[row][col] = '*'
                        _change = True
                elif _grid[row][col] is 0:
                    """
                              (x2, y2) 0
                                  |
                    1 (x2,y2) -- x,y -- (x2,y2) 3
                                  |
                              (x2, y2) 2

                    go through each of the position. check its not out of grid and grid position is not blocked
                    """
                    for _index in range(len(_delta)):
                        new_row = row + _delta[_index][0]
                        new_col = col + _delta[_index][1]
                        row_boundary = 0 <= new_row < len(_grid)
                        col_boundary = 0 <= new_col < len(_grid[0])
                        if row_boundary and col_boundary and _grid[new_row][new_col] is 0:
                            new_value = _value[new_row][new_col] + _cost
                            """
                                |   |                   |   |
                              __________             __________
                                |   | 99     --->       |   | 1
                              __________             __________
                                | 99| 0                 | 1 | 0
                            """
                            if new_value < _value[row][col]:
                                _value[row][col] = new_value
                                _policy[row][col] = delta_name[_index]
                                _change = True
    return _value, _policy


def stochastic_policy(_grid, _goal, _cost, _delta, _collision_cost, _success_prob):
    _value = [[_collision_cost for row in range(len(_grid[0]))] for col in range(len(_grid))]
    _policy = [[' ' for row in range(len(_grid[0]))] for col in range(len(_grid))]
    """
    failure probability
    divided in 2 parts because there can be 2 possible non-original moves
    """
    _failure_prob = (1.0 - _success_prob) / 2.0
    _change = True
    while _change is True:
        _change = False
        for row in range(len(_grid)):
            for col in range(len(_grid[0])):
                if row is _goal[0] and col is _goal[1]:
                    if _value[row][col] > 0:
                        _value[row][col] = 0
                        _policy[row][col] = '*'
                        _change = True
                elif _grid[row][col] is FREE:
                    for _index in range(len(_delta)):
                        new_value = _cost
                        """
                        for every move using delta,
                        there could be 3 possible movements because now movement is not exact
                        it is stochastic and possible outcome of each action is not with 1.0 probability
                        """
                        for _stochastic_index in range(-1, 2):
                            """
                            new_index is one of the stochastic index
                            for every new_index get the location
                            and assign it the value based on where it is
                            so if stochastic index is 0 then its success-probability else its failure-probability
                            """
                            new_index = (_index + _stochastic_index) % len(_delta)
                            new_row = row + _delta[new_index][0]
                            new_col = col + _delta[new_index][1]

                            _p = _success_prob if _stochastic_index is 0 else _failure_prob

                            row_boundary = 0 <= new_row < len(_grid)
                            col_boundary = 0 <= new_col < len(_grid[0])
                            """
                            if it is within boundaries then no penalty
                            if its hitting the wall i.e. going outside the boundary then apply penalty
                            """
                            if row_boundary and col_boundary and _grid[new_row][new_col] is FREE:
                                new_value += _value[new_row][new_col] * _p
                            else:
                                new_value += _collision_cost * _p
                        """
                        update the value of the current position
                        """
                        if new_value < _value[row][col]:
                            _value[row][col] = new_value
                            _policy[row][col] = delta_name[_index]
                            _change = True
    return _value, _policy


[g, x, y, action, expand] = breadth_first_search(grid,
                                                 delta,
                                                 init,
                                                 goal,
                                                 cost)

[a_f, a_g, a_x, a_y, a_action, a_expand] = heuristic_search(grid,
                                                            heuristic,
                                                            init,
                                                            goal,
                                                            cost)

[dynamic_value, dynamic_policy] = dynamic_policy(grid_dynamic,
                                                 goal_dynamic,
                                                 cost,
                                                 delta)

[stochastic_value, stochastic_policy] = stochastic_policy(grid_dynamic,
                                                          goal_dynamic,
                                                          cost,
                                                          delta,
                                                          collision_cost,
                                                          success_prob)

print("\nsearch value : ")
for _v in expand:
    print(_v)

print("\nheuristic value : ")
for _e in a_expand:
    print(_e)

print("\nsearch policy: ")
trace_search(grid, init, goal, action)

print("\ndynamic value: ")
for _v in dynamic_value:
    print(_v)

print("\ndynamic policy: ")
for _v in dynamic_policy:
    print(_v)

print("\nstochastic value: ")
for _v in stochastic_value:
    print(_v)

print("\nstochastic policy: ")
for _v in stochastic_policy:
    print(_v)


def monty_hall_problem():
    my_door = randint(1, 3)
    number_of_doors = 3
    other_doors = range(1, my_door) + range(my_door + 1, number_of_doors + 1)
    monty_door = choice(other_doors)
    p_init = 1. / number_of_doors
    p = [p_init for col in range(number_of_doors)]
    for index in range(number_of_doors):
        if index != (my_door - 1):
            p[index] += (1 / float(number_of_doors))
    p[monty_door - 1] = 0

    print("my door: ", my_door)
    print("monty door: ", monty_door)
    return p.index(max(p)) + 1


best_door = monty_hall_problem()
print(best_door, "has highest probability")
