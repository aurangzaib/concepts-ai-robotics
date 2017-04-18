p = [0, 0, 0, 1, 0]
pHit = 0.8
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1
world = ['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
motions = [1, 1]


# sense function
# increases entropy
def sense(_p, measurement):
    q = []
    for sense_index in range(len(_p)):
        if world[sense_index] is measurement:
            q.append(_p[sense_index] * pHit)
        else:
            q.append(_p[sense_index] * pMiss)
    return q


#  algorithm from sebastian
# decreases entropy
def move_algo_1(_p, _U):
    q = []
    for move_index in range(len(_p)):
        q.append(_p[(move_index - _U) % len(_p)])
    return q


#  my algorithm
# decreases entropy
def move_algo_2(p, move_point):
    q = [None] * len(p)
    for move_algo_2_index in range(len(p)):
        q[move_point] = (p[move_algo_2_index])
        move_point = (move_point + 1) % len(p)
    return q


# my algorithm
def move_algo_with_inaccurate_movement(p, move_point, p_exact=0.8, p_undershoot=0.1, p_overshoot=0.1):
    q = [None] * len(p)
    for move_algo_2_index in range(len(p)):
        q[move_point] = (p[move_algo_2_index]) * p_exact
        q[move_point] += (p[(move_algo_2_index + 1) % len(p)]) * p_undershoot
        q[move_point] += (p[(move_algo_2_index - 1) % len(p)]) * p_overshoot
        move_point = (move_point + 1) % len(p)
    return q


# sebastian algorithm
# decreases entropy
def move_algo_with_inaccurate_movement_2(p, U):
    q = []
    for index in range(len(p)):
        # adjust inaccurate robot movement
        value_exact = p[(index - U) % len(p)] * pExact  # current position
        value_overshoot = p[(index - U - 1) % len(p)] * pOvershoot  # previous position
        value_undershoot = p[(index - U + 1) % len(p)] * pUndershoot  # next position

        q.append(value_exact + value_overshoot + value_undershoot)
    return q


"""
    Limit Distribution:
    if a robot keeps moving with 
    inaccuracy in its movements not
    corrected, in the end it will be
    on uniform distribution, called
    state of maximal uncertainty.   
    
    at this point, robot doesn't know 
    exactly where it is. it thinks 
    it can be on aby of the possible 
    positions.
"""


# increasing the value of T
# make the robot more uncertain
# and the result will always tends
# towards uniform distribution
def perform_move_cycle():
    q = p[:]
    T = 10000
    for index in range(T):
        q = move_algo_with_inaccurate_movement(q, 1)
    print q


"""
    entropy:
    measure of info a distribution has
    update(move)    --> loses entropy
    measure(sense)  --> gains entropy
"""


def perform_localization():
    _p = [0.2, 0.2, 0.2, 0.2, 0.2]
    q = _p[:]
    for innerIndex in range(len(motions)):
        # measure
        q = sense(q, measurements[innerIndex])
        # then move
        q = move_algo_with_inaccurate_movement(q, motions[innerIndex])
    _p = q[:]
    return _p


"""
LOCALIZATION -- NXN
"""


def sense_n_by_n(_p, _colors, _measurement, _sensor_right):
    _sensor_wrong = 1. - _sensor_right
    _q = [[0 for row in range(len(_p[0]))] for col in range(len(_p))]
    normalize_factor = 0.0
    """
    find probabilities of each cell
    based on measurements given
    """
    for outer_index in range(len(_colors)):
        for inner_index in range(len(_colors[outer_index])):
            if _colors[outer_index][inner_index] is _measurement:
                _q[outer_index][inner_index] = _p[outer_index][inner_index] * _sensor_right
            else:
                _q[outer_index][inner_index] = _p[outer_index][inner_index] * _sensor_wrong
            normalize_factor += _q[outer_index][inner_index]
    """
    normalize the values so that 
    sum is always 1
    """
    for outer_index in range(len(_colors)):
        for inner_index in range(len(_colors[outer_index])):
            _q[outer_index][inner_index] /= normalize_factor
    """
    return the computed probabilities
    """
    return _q


def move_n_by_n(_p, _motions, _p_move):
    _p_stay = 1. - _p_move
    # initialize with 0 having same dimensions as
    # as p. deepcopy(p) could also be used but it would be slow
    __q = [[0 for row in range(len(_p[0]))] for col in range(len(p))]
    # x and y movements
    motions_x = _motions[1]
    motions_y = _motions[0]

    for vertical_index in range(len(_p)):
        for horizontal_index in range(len(_p[vertical_index])):
            # probability that the robot moved
            # e.g. p[0][0] copied to q[0][1]
            move_value = _p[vertical_index][horizontal_index] * _p_move

            # probability that the robot didn't move
            # e.g. p[0][1] copied to q[0][1]
            not_move_value = _p[motions_y % len(_p)][motions_x % len(_p[vertical_index])] * _p_stay

            # __q now have prob of moved and not_moved
            __q[motions_y][motions_x] = move_value + not_move_value

            # increment x index
            motions_x = (motions_x + 1) % len(_p[vertical_index])

        # increment y index
        motions_y = (motions_y + 1) % len(_p)
    return __q


def show_nxn(p):
    for outer_index in range(len(p)):
        for inner_index in range(len(p[outer_index])):
            print p[outer_index][inner_index], " ",
        print


colors_n_by_n = [['R', 'G', 'G', 'R', 'R'],
                 ['R', 'R', 'G', 'R', 'R'],
                 ['R', 'R', 'G', 'G', 'R'],
                 ['R', 'R', 'R', 'R', 'R']]
measurements_n_by_n = ['G', 'G', 'G', 'G', 'G']
motions_nxn = [[0, 0], [0, 1], [1, 0], [1, 0], [0, 1]]
sensor_right = .7
p_move = .8


def perform_localization_nxn(_colors, _measurements, _motions, _s_right, _p_move):
    # generating uniform distribution
    p_init = 1.0 / float(len(_colors)) / float(len(_colors[0]))
    q = [[p_init for row in range(len(_colors[0]))] for col in range(len(_colors))]

    for index in range(len(_measurements)):
        q = move_n_by_n(q, _motions[index], _p_move)
        q = sense_n_by_n(q, _colors, _measurements[index], _s_right)
    return q


show_nxn(perform_localization_nxn(colors_n_by_n, measurements_n_by_n, motions_nxn, sensor_right, p_move))
