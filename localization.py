p = [0.2, 0.2, 0.2, 0.2, 0.2]
pHit = 0.9
pMiss = 0.1
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1
world = ['green', 'green', 'red', 'green', 'red']
measurements = ['red']
motions = [1, 1]


# sense function
# increases entropy
def sense(_p, measurement):
    q = []
    for sense_index in range(len(_p)):
        value = p[sense_index] * pHit if (world[sense_index] is measurement) else p[sense_index] * pMiss
        q.append(value)
    normalizer = sum(q)
    for index in range(len(q)):
        q[index] /= normalizer
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
    for _index in range(len(p)):
        q[move_point] = (p[_index])
        move_point = (move_point + 1) % len(p)
    return q


# my algorithm
def move_algo_with_inaccurate_movement(p, move_point, p_exact=0.8, p_undershoot=0.1, p_overshoot=0.1):
    q = [None] * len(p)
    for _index in range(len(p)):
        q[move_point] = (p[_index]) * p_exact
        q[move_point] += (p[(_index + 1) % len(p)]) * p_undershoot
        q[move_point] += (p[(_index - 1) % len(p)]) * p_overshoot
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
    the inaccuracy in its movements not
    corrected, in the end it will be
    on uniform distribution, called
    state of maximal uncertainty.   
    
    at this point, robot doesn't know 
    exactly where it is. it thinks 
    it can be on any of the possible 
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


"""
    entropy: measure of info a distribution has
    update(move)    --> loses entropy --> info decreases
    measure(sense)  --> gains entropy --> info increases
    
          (sense - product - bayes rule)   -->   (move - convolution - total prob. theorem)
                                           <--    
                ^
                |
                |
                |
          (initial belief)
    normally uniform distribution
    
    sense --> probability distribution, world values, current measurement, sensor right probability
    move  --> probability distribution, motion, move probability
"""


def perform_localization(_measurements, _motions):
    _p = [0.2, 0.2, 0.2, 0.2, 0.2]
    q = _p[:]
    for innerIndex in range(len(_motions)):
        # measure
        q = sense(q, _measurements[innerIndex])
        # then move
        q = move_algo_with_inaccurate_movement(q, _motions[innerIndex])
    _p = q[:]
    return _p


"""
LOCALIZATION -- NXN
"""


def sense_n_by_n(_p, _colors, _measurement, _sensor_right):
    _sensor_wrong = 1. - _sensor_right
    """
    sensor_right --> pHit
    sensor_wrong --> pMiss
    """
    _q = [[0 for row in range(len(_p[0]))] for col in range(len(_p))]
    normalize_factor = 0.0
    """
    find probabilities of each cell
    based on measurements given
    """
    for outer_index in range(len(_colors)):
        for inner_index in range(len(_colors[0])):
            _condition = _colors[outer_index][inner_index] is _measurement
            _right = _p[outer_index][inner_index] * _sensor_right
            _wrong = _p[outer_index][inner_index] * _sensor_wrong

            _q[outer_index][inner_index] = _right if _condition else _wrong
            normalize_factor += _q[outer_index][inner_index]
    """
    normalize the values so that 
    sum is always 1
    """
    for outer_index in range(len(_colors)):
        for inner_index in range(len(_colors[0])):
            _q[outer_index][inner_index] /= normalize_factor
    """
    return the computed probabilities
    """
    return _q


def move_n_by_n(_p, _motions, _p_move):
    _p_stay = 1. - _p_move
    """
    initialize with 0 having same dimensions as p. 
    deepcopy(p) could also be used
    """
    __q = [[0 for row in range(len(_p[0]))] for col in range(len(_p))]
    """
    handling cyclic index case
    user may have given wrong value to be moved
    """
    move_row = _motions[0] % len(_p)
    move_col = _motions[1] % len(_p[0])
    for index_row in range(len(_p)):
        for index_col in range(len(_p[0])):
            """
            probabilities that the robot moved and not moved
            e.g. p[0][0] copied to q[0][1]
            """
            move_value = _p[index_row][index_col] * _p_move
            not_move_value = _p[move_row][move_col] * _p_stay
            """
            __q now have prob of moved and not_moved
            """
            __q[move_row][move_col] = move_value + not_move_value
            """
            increment x index
            """
            move_col = (move_col + 1) % len(_p[0])
        """
        increment y index
        """
        move_row = (move_row + 1) % len(_p)
    return __q


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
    # p_init = 1/(row*col)
    p_init = 1.0 / float(len(_colors) * len(_colors[0]))
    q = [[p_init for row in range(len(_colors[0]))] for col in range(len(_colors))]
    for index in range(len(_measurements)):
        q = move_n_by_n(q, _motions[index], _p_move)
        q = sense_n_by_n(q, _colors, _measurements[index], _s_right)
    for _q in q:
        print _q
    return q


q = perform_localization_nxn(colors_n_by_n,
                             measurements_n_by_n,
                             motions_nxn,
                             sensor_right,
                             p_move)
