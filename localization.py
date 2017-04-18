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


