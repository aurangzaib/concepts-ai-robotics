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
        robot will move always. but may not to desired location
        """
        q = [0 for col in range(len(p))]
        move_point = move_point % len(p)
        for _index in range(len(p)):
            q[move_point] += (p[_index]) * p_exact
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
        q = [None for row in range(len(p))]
        move_point = move_point % len(p)
        for index in range(len(p)):
            move = p[index] * p_move
            stay = p[move_point] * p_stay
            q[move_point] = move + stay
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
    def sense(p, world_map, sensor_data, p_hit=0.9):
        """
        we compare the sensor measurement value with the world values
        if sensor measurement measurement
        """
        q = [0 for col in range(len(p))]
        p_miss = 1. - p_hit
        for i in range(len(p)):
            hit_condition = world_map[i] is sensor_data
            q[i] = p[i] * p_hit if hit_condition else p[i] * p_miss
        normalizer = sum(q)
        for index in range(len(q)):
            q[index] /= normalizer
        return q

    @staticmethod
    def sense_nxn(_p, world_map, sensor_data, _sensor_right):
        _sensor_wrong = 1. - _sensor_right
        """
        sensor_right --> pHit
        sensor_wrong --> pMiss
        """
        q = [[0 for row in range(len(_p[0]))] for col in range(len(_p))]
        normalize_factor = 0.0
        """
        find probabilities of each cell
        based on measurements given
        """
        for row in range(len(world_map)):
            for col in range(len(world_map[0])):
                condition = world_map[row][col] is sensor_data
                right = _p[row][col] * _sensor_right
                wrong = _p[row][col] * _sensor_wrong

                q[row][col] = right if condition else wrong
            normalize_factor += sum(q[row])
        """
        normalize the values so that
        sum is always 1
        """
        for row in range(len(world_map)):
            for col in range(len(world_map[0])):
                q[row][col] /= normalize_factor
        """
        return the computed probabilities
        """
        return q

    @staticmethod
    def localize(_measurements, world_values, _motions):
        p = [0.2, 0.2, 0.2, 0.2, 0.2]
        q = p[:]
        for index in range(len(_motions)):
            # measure -- sense
            q = Localization.sense(q, world_values, _measurements[index])
            # predict -- move
            q = Localization.move_overshoot(q, _motions[index])
        p = q[:]
        return p

    @staticmethod
    def localize_nxn(_colors, _measurements, _motions, _s_right, _p_move):
        """
        motions is a list of tuples of x, y motions
        colors is a matrix of world map
        measurement is a list of sensor data
        s_right is a float
        p_move is a float
        q is a matrix with same dimension as colors matrix
        """

        # starting from uniform/limit distribution
        p_init = 1.0 / float(len(_colors) * len(_colors[0]))
        q = [[p_init for col in range(len(_colors[0]))] for row in range(len(_colors))]
        for motion, measurement in zip(_motions, _measurements):
            # predict
            q = Localization.move_nxn(q, motion, _p_move)
            # measure
            q = Localization.sense_nxn(q, _colors, measurement, _s_right)
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
        cycles = 10000
        for index in range(cycles):
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

_q_ = Localization.localize_nxn(colors_n_by_n,
                                measurements_n_by_n,
                                motions_nxn,
                                sensor_right,
                                p_move)
