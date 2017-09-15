"""
Limit Distribution:
if a robot keeps moving with the inaccuracy in its movements not
corrected, in the end it will be on uniform distribution, called
state of maximal uncertainty. at this point, robot doesn't know exactly where it is.
it thinks it can be on any of the possible positions.

entropy: measure of info a distribution has
update(move)    --> loses entropy --> info decreases
measure(sense)  --> gains entropy --> info increases

      (sense|product|bayes rule)   -->   (move|convolution|total prob. theorem)
                                   <--
            ^
            |
      (initial belief) normally uniform distribution

sense --> probability distribution, world values, current measurement, sensor right probability
move  --> probability distribution, motion, move probability
"""


class Localization:
    @staticmethod
    def move(p, move_point):
        """
        move in vector
        """
        q = [None] * len(p)
        for _index in range(len(p)):
            q[move_point] = (p[_index])
            move_point = (move_point + 1) % len(p)
        return q

    @staticmethod
    def move_overshoot(p, move_point, p_exact=0.8, p_undershoot=0.1, p_overshoot=0.1):
        """
        move with overshoot and undershoot
        robot will move always. but may not desired location
        """
        q = [None] * len(p)  # q = [None for row in range(len(p))]
        for _index in range(len(p)):
            q[move_point] = (p[_index]) * p_exact
            q[move_point] += (p[(_index + 1) % len(p)]) * p_undershoot
            q[move_point] += (p[(_index - 1) % len(p)]) * p_overshoot
            move_point = (move_point + 1) % len(p)
        return q

    @staticmethod
    def move_stochastic(p, move_point, p_move):
        """
        there is possibility that robot may not move at all
        """
        p_stay = 1. - p_move
        q = [None] * len(p)
        move_point = move_point % len(p)
        for index in range(len(p)):
            move = p[index] * p_move
            stay = p[move_point] * p_stay
            q[move] = move + stay
            move_point = (move_point + 1) % len(p)
        return q

    @staticmethod
    def move_nxn(p, _motions, _p_move=0.8):
        p_stay = 1.0 - p_move
        # initialize q with 0s with same dimensions as p
        q = [[0 for col in range(len(p[0]))] for row in range(len(p))]
        # handle boundary cases
        ur, uc = _motions[0] % len(p), _motions[1] % len(p[0])
        for r in range(len(p)):
            for c in range(len(p[0])):
                # robot moved
                move = p[r][c] * p_move
                # robot not moved
                stay = p[ur][uc] * p_stay
                # combined probability
                q[ur][uc] = move + stay
                # next col
                uc = (uc + 1) % len(p[0])
            # next row
            ur = (ur + 1) % len(p)
        return q

    @staticmethod
    def sense(p, world_values, measurement, p_hit=0.9):
        """
        we compare the sensor measurement value with the world values
        if sensor measurement measurement
        """
        q = []
        p_miss = 1. - p_hit
        for i in range(len(p)):
            value = p[i] * p_hit if (world_values[i] is measurement) else p[i] * p_miss
            q.append(value)
        normalizer = sum(q)
        for index in range(len(q)):
            q[index] /= normalizer
        return q

    @staticmethod
    def sense_nxn(_p, _colors, _measurement, _sensor_right):
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

    @staticmethod
    def localize(_measurements, world_values, _motions):
        p = [0.2, 0.2, 0.2, 0.2, 0.2]
        q = p[:]
        for innerIndex in range(len(_motions)):
            # measure
            q = Localization.sense(q, world_values, _measurements[innerIndex])
            # then move
            q = Localization.move_overshoot(q, _motions[innerIndex])
        p = q[:]
        return p

    @staticmethod
    def localize_nxn(_colors, _measurements, _motions, _s_right, _p_move):
        # generating uniform distribution
        p_init = 1.0 / float(len(_colors) * len(_colors[0]))
        q = [[p_init for col in range(len(_colors[0]))] for row in range(len(_colors))]
        for index in range(len(_measurements)):
            q = Localization.move_nxn(q, _motions[index], _p_move)
            q = Localization.sense_nxn(q, _colors, _measurements[index], _s_right)
        for _q in q:
            print(_q)
        return q

    @staticmethod
    def move_cycle(p):
        """
        increasing the value of T
        make the robot more uncertain
        and the result will always tends
        towards uniform distribution
        """
        q = p[:]
        T = 10000
        for index in range(T):
            q = Localization.move_overshoot(q, 1)


probabilities = [0.2, 0.2, 0.2, 0.2, 0.2]
world = ['green', 'green', 'red', 'green', 'red']
measurements = ['red']
motions = [1, 1]

colors_n_by_n = [['R', 'G', 'G', 'R', 'R'],
                 ['R', 'R', 'G', 'R', 'R'],
                 ['R', 'R', 'G', 'G', 'R'],
                 ['R', 'R', 'R', 'R', 'R']]
measurements_n_by_n = ['G', 'G', 'G', 'G', 'G']
motions_nxn = [[0, 0], [0, 1], [1, 0], [1, 0], [0, 1]]
sensor_right = .7
p_move = .8

q = Localization.localize_nxn(colors_n_by_n,
                              measurements_n_by_n,
                              motions_nxn,
                              sensor_right,
                              p_move)
