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
delta = [[-1, 0],
         [0, -1],
         [1, 0],
         [0, 1]]
init = [0, 0]

goal = [len(grid) - 1, len(grid[0]) - 1]
cost = 1
delta_name = ['^', '<', 'v', '>']


def path_find(_grid, _heuristic, _goal, _delta, _init, _cost):
    _closed = [[0 for row in range(len(_grid[0]))] for col in range(len(_grid))]
    _action = [[-1 for row in range(len(_grid[0]))] for col in range(len(_grid))]
    _expand = [[-1 for row in range(len(_grid[0]))] for col in range(len(_grid))]
    init[0] = 0
    init[1] = 0
    _x = _init[0]
    _y = _init[1]
    _g = 0
    _f = _g + _heuristic[_x][_y]
    _open = [[_f, _g, _x, _y]]
    _closed[_x][_y] = 1
    _resign = False
    _found = False
    _count = 0
    while _resign is False and _found is False:
        if len(_open) is 0:
            _resign = True
        else:
            _open.sort()
            _open.reverse()
            _next = _open.pop()
            _f = _next[0]
            _g = _next[1]
            _x = _next[2]
            _y = _next[3]
            _expand[_x][_y] = _count
            _count += 1
            if _x is _goal[0] and _y is _goal[1]:
                _found = True
            else:
                for _index in range(len(_delta)):
                    _x2 = _x + _delta[_index][0]
                    _y2 = _y + _delta[_index][1]
                    x_boundary_condition = _x2 >= 0 and _x2 < len(_grid)
                    y_boundary_condition = _y2 >= 0 and _y2 < len(_grid[0])
                    if x_boundary_condition and y_boundary_condition:
                        if _closed[_x2][_y2] is 0 and _grid[_x2][_y2] is 0:
                            _g2 = _g + _cost
                            _f2 = _g2 + _heuristic[_x2][_y2]
                            _closed[_x2][_y2] = 1
                            _open.append([_f2, _g2, _x2, _y2])
                            _action[_x2][_y2] = _index
    return [_f, _g, _x, _y, _expand, _action]


def path_trace(_grid, _action, _delta, _delta_name, _init, _goal):
    _policy = [[' ' for row in range(len(_grid[0]))] for col in range(len(_grid))]
    _x = _goal[0]
    _y = _goal[1]
    _policy[_x][_y] = '*'
    while _x != _init[0] or _y != _init[1]:
        _x2 = _x - _delta[_action[_x][_y]][0]
        _y2 = _y - _delta[_action[_x][_y]][1]
        _policy[_x2][_y2] = _delta_name[_action[_x][_y]]
        _x = _x2
        _y = _y2
    return _policy


def dynamic_optimum_policy(_grid, _goal, _delta, _delta_name, _init, _cost):
    _value = [[99 for row in range(len(_grid[0]))] for col in range(len(_grid[0]))]
    _policy = [[' ' for row in range(len(_grid[0]))] for col in range(len(_grid))]
    _change = True
    while _change is True:
        _change = False
        for _x in range(len(grid)):
            for _y in range(len(grid[0])):
                if _x is _goal[0] and _y is _goal[1]:
                    if (_value[_x][_y] > 0):
                        _value[_x][_y] = 0
                        _change = True
                        _policy[_x][_y] = '*'
                elif _grid[_x][_y] is 0:
                    for _index in range(len(_delta)):
                        _x2 = _x + _delta[_index][0]
                        _y2 = _y + _delta[_index][1]
                        _x_boundary = _x2 >= 0 and _x2 < len(_grid)
                        _y_boundary = _y2 >= 0 and _y2 < len((_grid[0]))
                        if _x_boundary and _y_boundary:
                            if _grid[_x][_y] is 0:
                                _next_value = _value[_x2][_y2] + _cost
                                if _next_value < _value[_x][_y]:
                                    _value[_x][_y] = _next_value
                                    _policy[_x][_y] = _delta_name[_index]
                                    _change = True
    return [_value, _policy]


[f, g, x, y, expand, action] = path_find(grid, heuristic, goal, delta, init, cost)
policy = path_trace(grid, action, delta, delta_name, init, goal)
[optimum_value, optimum_policy] = dynamic_optimum_policy(grid, goal, delta, delta_name, init, cost)
print "\nexpand: "
for _e in expand: print _e
print "\npolicy: "
for _p in policy: print _p
print "\ndynamic optimum policy: "
for _p in optimum_policy: print _p
