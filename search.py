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

# Breadth First Planning
def search(_grid, _delta, _init, _goal, _cost):
    # initialize closed with zeros having same dimensions as grid
    _closed = [[0 for row in range(len(_grid[0]))] for col in range(len(_grid))]
    _expand = [[-1 for row in range(len(_grid[0]))] for col in range(len(_grid))]
    _action = [[-1 for row in range(len(_grid[0]))] for col in range(len(_grid))]
    _init[0] = 0
    _init[1] = 0
    x = _init[0]
    y = _init[1]
    g = 0
    _closed[_init[0]][_init[1]] = 1
    # note that g is 0th element. because its the criteria for sorting
    _open = [[g, x, y]]
    count = 0
    resign = False
    found = False
    while resign is False and found is False:
        if len(_open) is 0:
            resign = True
        else:
            _open.sort()
            _open.reverse()
            _next = _open.pop()  # remove the element with lowest g value
            g = _next[0]
            x = _next[1]
            y = _next[2]
            _expand[x][y] = count
            count += 1
            if _goal[0] is x and _goal[1] is y:
                found = True
            else:
                for index in range(len(_delta)):
                    # move forward to the next location
                    x2 = x + _delta[index][0]
                    y2 = y + _delta[index][1]
                    _boundary_condition = 0 <= x2 < len(_grid) and 0 <= y2 < len(_grid[0])
                    _grid_is_free = _closed[x2][y2] is FREE and _grid[x2][y2] is FREE
                    if _boundary_condition:
                        if _grid_is_free:
                            g2 = g + _cost
                            _open.append([g2, x2, y2])
                            _closed[x2][y2] = 1
                            """
                            saving which action from delta is performed used when tracing the policy
                            (x2,y2) are used instead of (x,y)
                            (x, y) are values of all possible tried combinations
                            (x2, y2) are values of the combination we used in the end
                            """
                            _action[x2][y2] = index
    if resign is True:
        print 'Fail'
    elif found is True:
        return [g, x, y, action, _expand]
    else:
        return 'Unknown Error'

# A* Planning
def heuristic_search(grid, heuristic, init, goal, cost):
    # initialize with same dimensions as grid
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    expand = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    action = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    """
    A* method can reduce number of expanded nodes by calculating the new f value
    f = g + h(x,y)
    so whenever it has choice between 2 options it will select one with smaller f value but A* method is not
    efficient when both options have same f values. try using 2nd col as: [0 1 1 1 1 1] instead of [0 1 1 1 1 1]
    and A* won't work  very well because there will be several options with same f values.
    
    run with:
    grid = [[0, 0, 0, 0, 0, 0], [0, 1, 1, 1, 1, 0], [0, 1, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]
    and see how A* and normal expan grid method work
    expansion grid method will explore a lot of places.
     
    A* is very good in real world where we have dead end obstacles.
    this grid based A* id different for real world cars as cars can turn at an angle.
    """
    init[0] = 0
    init[1] = 0
    x = init[0]
    y = init[1]
    g = 0
    h = heuristic[x][y]
    f = g + h
    closed[init[0]][init[1]] = 1
    # note that f is 0th element. because its the criteria for sorting
    open = [[f, g, x, y]]
    count = 0
    resign = False
    found = False
    while resign is False and found is False:
        if len(open) is 0:
            resign = True
        else:
            open.sort()
            open.reverse()
            _next = open.pop()  # remove the element with lowest f value
            f = _next[0]
            g = _next[1]
            x = _next[2]
            y = _next[3]
            expand[x][y] = count
            count += 1
            if goal[0] is x and goal[1] is y:
                found = True
            else:
                for index in range(len(delta)):
                    """
                    move forward to the next location
                    and get x,y of the expanded node
                    """
                    x2 = x + delta[index][0]
                    y2 = y + delta[index][1]
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
        print 'Fail'
    elif found is True:
        return [f, g, x, y, action, expand]
    else:
        return 'Unknown Error'


def trace_search(grid, init, goal, _action):
    policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    # mark the goal coordinates as *
    __x = goal[0]
    __y = goal[1]
    policy[__x][__y] = '*'
    while __x != init[0] or __y != init[1]:
        """
        we took next positions when running find policy(path) algorithm as:
            x2 = x + delta(index)(0); y2 = y + delta(index)(1)
        here we will go in reverse direction and perform the recursion
            x2 = x - delta(action(x)(y))(0); y2 = y - delta(action(x)(y))(1)
        where x2,y2 are original coord.; x,y are current coord.
        """
        x2 = __x - delta[_action[__x][__y]][0]
        y2 = __y - delta[_action[__x][__y]][1]
        """
        now we will draw the character based on which action was performed
        action[x][y] is the index from delta and every delta element has its assoc. character in the delta_name vector
        """
        policy[x2][y2] = delta_name[_action[__x][__y]]
        """
        now save update x, y with x2, y2; recursion
        """
        __x = x2
        __y = y2

    for _index in range(len(policy)):
        print policy[_index]
    return policy

# Optimal Policy on each Node or Location
def dynamic_policy(_grid, _goal, _cost, _delta):
    # initialize value with 99
    _value = [[100 for row in range(len(_grid[0]))] for col in range(len(_grid))]
    _policy = [[' ' for row in range(len(_grid[0]))] for col in range(len(_grid))]
    _change = True
    """
    for x in rows
        for y in cols
            if x,y match with goals then mark value(x,y) as 0
            for k in delta
                x2 -> delta(k)(0) and y2 -> delta(k)(0)
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
        for _x in range(len(_grid)):
            # horizontal loop
            for _y in range(len(_grid[0])):
                # use debugger at this point to see how its proceeding
                """
                if we have reached on the goal position and value is larger. then assign it 0.
                goal in dynamic programming is 0
                """
                if _x is _goal[0] and _y is _goal[1]:
                    if _value[_x][_y] > 0:
                        _value[_x][_y] = 0
                        _policy[_x][_y] = '*'
                        _change = True
                elif _grid[_x][_y] is 0:
                    """
                              (x2, y2) 0
                                  |      
                    3 (x2,y2) -- x,y -- (x2,y2) 2            
                                  | 
                              (x2, y2) 1
                    
                    go through each of the position. check its not out of grid and grid position is not blocked
                    """
                    for _index in range(len(_delta)):
                        _x2 = _x + _delta[_index][0]
                        _y2 = _y + _delta[_index][1]
                        if 0 <= _x2 < len(grid) and 0 <= _y2 < len(grid[0]):
                            if _grid[_x2][_y2] is FREE:
                                _v2 = _value[_x2][_y2] + _cost
                                """
                                    |   |                   |   |
                                  __________             __________
                                    |   | 99     --->       |   | 1
                                  __________             __________      
                                    | 99| 0                 | 1 | 0
                                """
                                if _v2 < _value[_x][_y]:
                                    _value[_x][_y] = _v2
                                    _policy[_x][_y] = delta_name[_index]
                                    _change = True

    return [_value, _policy]

# Taking in account the probability of Robot movement and Wall Hitting Penalty
def stochastic_policy(_grid, _goal, _cost, _delta, _collision_cost, _success_prob):
    # initialize value with 1000
    _value = [[1000 for row in range(len(_grid[0]))] for col in range(len(_grid))]
    _policy = [[' ' for row in range(len(_grid[0]))] for col in range(len(_grid))]
    """
    failure probability
    divided in 2 parts because there can be 2 possible non-original moves 
    """
    _failure_prob = (1.0 - _success_prob) / 2.0
    _change = True
    while _change is True:
        _change = False
        for _x in range(len(_grid)):
            for _y in range(len(_grid[0])):
                if _x is _goal[0] and _y is _goal[1]:
                    if _value[_x][_y] > 0:
                        _value[_x][_y] = 0
                        _policy[_x][_y] = '*'
                        _change = True
                elif _grid[_x][_y] is FREE:
                    for _index in range(len(_delta)):
                        _v2 = _cost
                        """
                        for every move using delta,
                        there could be 3 possible movements because now movement is not exact
                        it is stochastic and possible outcome of each action is not with 1.0 probability
                        """
                        for _stochastic_index in range(-1, 2):
                            """
                            __index is one of the stochastic index
                            for every __index get the location
                            and assign it the value based on where it is
                            so if stochastic index is 0 then its success probability else its failure
                            """
                            __index = (_index + _stochastic_index) % len(_delta)
                            _x2 = _x + _delta[__index][0]
                            _y2 = _y + _delta[__index][1]
                            if _stochastic_index is 0:
                                _p = _success_prob
                            else:
                                _p = _failure_prob

                            _x_boundary = 0 <= _x2 < len(_grid)
                            _y_boundary = 0 <= _y2 < len(grid[0])
                            _grid_free = _grid[_x2][_y2] is FREE
                            """
                            if it is within boundaries then no penalty 
                            if its hitting the wall i.e. going outside the boundary
                            then apply penalty
                            """
                            if _x_boundary and _y_boundary and _grid_free:
                                _v2 += _value[_x2][_y2] * _p
                            else:
                                _v2 += _collision_cost * _p
                        """
                        update the value of the current position
                        """
                        if _v2 < _value[_x][_y]:
                            _value[_x][_y] = _v2
                            _policy[_x][_y] = delta_name[_index]
                            _change = True
    return [_value, _policy]


[g, x, y, action, expand] = search(grid,
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

print "\nsearch value : "
for _v in expand: print _v

print "\nheuristic value : "
for _e in a_expand: print _e

print "\nsearch policy: "
trace_search(grid, init, goal, action)

print "\ndynamic value: "
for _v in dynamic_value: print _v

print "\ndynamic policy: "
for _v in dynamic_policy: print _v

print "\nstochastic value: "
for _v in stochastic_value: print _v

print "\nstochastic policy: "
for _v in stochastic_policy: print _v
