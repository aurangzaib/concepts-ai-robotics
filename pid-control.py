from copy import deepcopy
from helper import *

path = [[0, 0], [0, 0.5], [0, 1], [0, 1.5], [0, 2], [0.5, 2],
        [1, 2], [1.5, 2], [2, 2], [2.5, 2], [3, 2], [3.5, 2],
        [4, 2], [4, 2.5], [4, 3], [4, 3.5], [4, 4]]

path_cyclic = [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [5, 0],
               [6, 0], [6, 1], [6, 2], [6, 3], [5, 3], [4, 3],
               [3, 3], [2, 3], [1, 3], [0, 3], [0, 2], [0, 1]]
fix_points = [1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0]


def smooth_gradient_descent(_path, weight_data=0.0, weight_smooth=0.1, tolerance=0.000001):
    _new_path = deepcopy(_path)
    _change = tolerance
    # keep iterating until change is smaller than the tolerance
    while _change >= tolerance:
        # set change to 0
        _change = 0.
        """
        loop over each path on (i,j) and apply gradient descent equations.
        each time update the difference and store in change variable.

        the value of weight_data determines the smoothness of the edges
    
        smaller the values the smoother the edges, tending towards straight line
        i.e. edges will have 0 angles for data_weight=0 i.e. straight line
    
        the larger the values the sharper the edges
        on data_weight=1 we will have 90 degree edges
        """
        for _i in range(1, len(_new_path) - 1):
            for _j in range(len(_new_path[0])):
                # values before updates
                _old_value = _new_path[_i][_j]
                # gradient and descent equations
                _gradient = weight_data * (_path[_i][_j] - _new_path[_i][_j])
                _descent = weight_smooth * (_new_path[_i + 1][_j] + _new_path[_i - 1][_j] - (2. * _new_path[_i][_j]))
                # save results of the equations
                _new_path[_i][_j] += _gradient + _descent
                # compare the outcomes of the gradient and descent results
                _change += abs(_new_path[_i][_j] - _old_value)
    return _new_path


def smooth_constrained_cyclic(_path, weight_data=0.07, weight_smooth=0.1, tolerance=0.000001):
    weight_constrained_smooth = 0.5 * weight_smooth
    _new_path = deepcopy(_path)
    _change = tolerance
    # keep iterating until change is smaller than the tolerance
    while _change >= tolerance:
        # set change to 0
        _change = 0.
        """
        loop over each path on (i,j) and apply gradient descent equations.
        each time update the difference and store in change variable.
        """
        for _i in range(len(_new_path)):
            if not fix_points[_i]:
                for _j in range(len(_new_path[0])):
                    # values before updates
                    _old_value = _new_path[_i][_j]

                    # gradient and descent equations
                    _gradient = weight_data * (_path[_i][_j] - _new_path[_i][_j])
                    _descent = weight_smooth * (
                        _new_path[(_i + 1) % len(_path)][_j] + _new_path[(_i - 1) % len(_path)][_j] - (
                            2. * _new_path[_i][_j]))

                    # constrained smoothing equations
                    constrained_smooth_backward = 2. * (_new_path[(_i - 1) % len(_path)][_j]) - \
                                                  _new_path[(_i - 2) % len(_path)][_j] - _new_path[_i][_j]
                    constrained_smooth_forward = 2. * (_new_path[(_i + 1) % len(_path)][_j]) - \
                                                 _new_path[(_i + 2) % len(_path)][_j] - _new_path[_i][_j]

                    # save results of the equations
                    _new_path[_i][_j] += _gradient + _descent
                    _new_path[_i][_j] += weight_constrained_smooth * constrained_smooth_backward
                    _new_path[_i][_j] += weight_constrained_smooth * constrained_smooth_forward

                    # compare the outcomes of the gradient and descent results
                    _change += abs(_new_path[_i][_j] - _old_value)
    return _new_path


def p_controller(_robot, tau_p, n=20, speed=1.0):
    _x_trajectory = []
    _y_trajectory = []
    for _index in range(n):
        # push current coordinates
        _x_trajectory.append(_robot.x)
        _y_trajectory.append(_robot.y)
        # steering for proportional
        steering = -tau_p * _robot.y
        _robot.move(steering, speed)
    return _x_trajectory, _y_trajectory


def pd_controller(_robot, tau_p, tau_d, n=20, speed=1.0):
    _x_trajectory = []
    _y_trajectory = []
    cte_old = _robot.y
    for _index in range(n):
        # push current coordinates
        _x_trajectory.append(_robot.x)
        _y_trajectory.append(_robot.y)
        # steering for proportional and differential
        _proportional = -tau_p * _robot.y
        _differential = -tau_d * (_robot.y - cte_old)
        # combined steering
        steering = _proportional + _differential
        # move the robot
        cte_old = _robot.y
        _robot.move(steering, speed)
    return _x_trajectory, _y_trajectory


def pid_controller(_robot, _radius, _params, n=100, _speed=1.0):
    _tau_p = _params[0]
    _tau_d = _params[1]
    _tau_i = _params[2]

    _x_trajectory = []
    _y_trajectory = []

    _error = 0
    _cte_old = _robot.y
    _cte_sum = 0

    for _index in range(n * 2):
        _cte = _robot.y
        # push current coordinates
        _x_trajectory.append(_robot.x)
        _y_trajectory.append(_robot.y)
        # accumulating all the error
        _cte_sum += _cte
        # steering for pid
        _proportional = -_tau_p * _cte
        _differential = -_tau_d * (_cte - _cte_old)
        _integral = -_tau_i * _cte_sum
        _steering = _proportional + _differential + _integral
        # save old value of the cte i.e. y before moving the robot
        _cte_old = _cte
        # move the robot with the steering and speed (how much distance)
        _robot.move(_steering, _speed)
        if _index >= n:
            _error += _cte ** 2
    return _x_trajectory, _y_trajectory, _error / n


