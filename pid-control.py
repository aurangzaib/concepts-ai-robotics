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
        for row in range(1, len(_new_path) - 1):
            for col in range(len(_new_path[0])):
                # values before updates
                _old_value = _new_path[row][col]
                # gradient and descent equations
                _gradient = _path[row][col] - _new_path[row][col]
                _descent = _new_path[row + 1][col] + _new_path[row - 1][col] - (2. * _new_path[row][col])
                # save results of the equations
                _new_path[row][col] += weight_data * _gradient + weight_smooth * _descent
                # compare the outcomes of the gradient and descent results
                _change += abs(_new_path[row][col] - _old_value)
    return _new_path


def smooth_constrained_cyclic(_path,
                              weight_data=0.07,
                              weight_smooth=0.1,
                              tolerance=0.000001):
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
        for row in range(len(_new_path)):
            if not fix_points[row]:
                for col in range(len(_new_path[0])):
                    # values before updates
                    _old_value = _new_path[row][col]

                    # gradient and descent equations
                    _gradient = _path[row][col] - _new_path[row][col]
                    _descent = _new_path[(row + 1) % len(_path)][col] + _new_path[(row - 1) % len(_path)][col]
                    _descent -= 2. * _new_path[row][col]

                    # constrained smoothing equations
                    constrained_smooth_backward = 2. * (_new_path[(row - 1) % len(_path)][col])
                    constrained_smooth_backward -= _new_path[(row - 2) % len(_path)][col]
                    constrained_smooth_backward -= _new_path[row][col]

                    constrained_smooth_forward = 2. * (_new_path[(row + 1) % len(_path)][col])
                    constrained_smooth_forward -= _new_path[(row + 2) % len(_path)][col]
                    constrained_smooth_forward -= _new_path[row][col]

                    # save results of the equations
                    _new_path[row][col] += weight_data * _gradient + weight_smooth * _descent
                    _new_path[row][col] += weight_constrained_smooth * constrained_smooth_backward
                    _new_path[row][col] += weight_constrained_smooth * constrained_smooth_forward

        # compare the outcomes of the gradient and descent results
        _change += abs(_new_path[row][col] - _old_value)
    return _new_path


def p_controller(_robot, tau_p, n=20, speed=1.0):
    _x_trajectory = []
    _y_trajectory = []

    for _index in range(n):
        cte = robot.y
        # push current coordinates
        _x_trajectory.append(_robot.x)
        _y_trajectory.append(_robot.y)
        # steering for proportional
        steering = -tau_p * cte
        _robot.move_simple(steering, speed)
    return _x_trajectory, _y_trajectory


def pd_controller(_robot, tau_p, tau_d, n=20, speed=1.0):
    _x_trajectory = []
    _y_trajectory = []
    cte = robot.y
    for _index in range(n):
        cte_old = cte
        cte = robot.y
        # push current coordinates
        _x_trajectory.append(_robot.x)
        _y_trajectory.append(_robot.y)
        # steering for proportional and differential
        _proportional = -tau_p * cte
        _differential = -tau_d * (cte - cte_old)
        # combined steering
        steering = _proportional + _differential
        # move the robot
        _robot.move_simple(steering, speed)
    return _x_trajectory, _y_trajectory


def pid_controller(_robot, _radius, _params, n=100, _speed=1.0):
    _tau_p = _params[0]
    _tau_d = _params[1]
    _tau_i = _params[2]

    _x_trajectory = []
    _y_trajectory = []

    _error = 0
    _cte = _robot.y
    _cte_sum = 0

    for _index in range(n * 2):
        # previous value
        _cte_old = _cte
        # current value
        _cte = robot.y
        # sum of cte
        _cte_sum += _cte
        # push current coordinates
        _x_trajectory.append(_robot.x)
        _y_trajectory.append(_robot.y)
        # accumulating all the error
        # steering for pid
        _proportional = -_tau_p * _cte
        _differential = -_tau_d * (_cte - _cte_old)
        _integral = -_tau_i * _cte_sum
        _steering = _proportional + _differential + _integral
        # save old value of the cte i.e. y before moving the robot
        # move the robot with the steering and speed (how much distance)
        _robot.move_simple(_steering, _speed)
        if _index >= n:
            _error += _cte ** 2
    return _x_trajectory, _y_trajectory, _error / n


def pid_controller_race_track(_robot, _radius, _params, n=100, _speed=1.0):
    _tau_p = _params[0]
    _tau_d = _params[1]
    _tau_i = _params[2]

    _x_trajectory = []
    _y_trajectory = []

    _error = 0
    _cte_old = _robot.cte(_radius)
    _cte_sum = 0

    for _index in range(n * 2):
        _cte = _robot.cte(_radius)
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
        _robot.move_simple(_steering, _speed)
        if _index >= n:
            _error += _cte ** 2
    return _x_trajectory, _y_trajectory, _error / n


def twiddle(_controller=pid_controller, _radius=0., _tolerance=0.2):
    _number_of_params = 3
    _p = [0. for row in range(_number_of_params)]
    _dp = [1. for row in range(_number_of_params)]
    """
    if we make _dp[2] = 0 i.e. we turn off the integral then the results will have drift again
    same goes for _dp[1]

    making _dp[any_one] = 0 tells to remove p, i or d
    that's why we initialize _dp with 1,1,1 always
    """
    _robot = make_robot()
    _x, _y, _best_error = _controller(_robot, _radius, _p)
    _iteration = 0
    while sum(_dp) > _tolerance:
        for _index in range(len(_p)):
            _p[_index] += _dp[_index]
            _robot = make_robot()
            _x, _y, _error = _controller(_robot, _radius, _p)
            """
            if we get low error
            then we will use it
            """
            if _error < _best_error:
                _best_error = _error
                _dp[_index] *= 1.1
            else:
                """
                otherwise we will try other direction
                (- _dp[_index]) will move to original position
                (- 2*_dp[_index]) will move to opposite direction
                """
                _p[_index] -= 2 * _dp[_index]
                _robot = make_robot()
                _x, _y, _error = _controller(_robot, _radius, _p)
                if _error < _best_error:
                    _best_error = _error
                    _dp[_index] *= 1.1
                else:
                    """
                    revert to the original
                    """
                    _p[_index] += _dp[_index]
                    _dp[_index] *= 0.9
        _iteration += 1
    return _p


# gives smooth path
path_smooth = smooth_gradient_descent(path, 0.08, 0.1)
# gives straight line connecting start and end points
path_straight = smooth_gradient_descent(path, 0, 0.1)
# no change
path_like_original = smooth_gradient_descent(path, 0.5, 0)
# cyclic paths with smoothing
path_cyclic_smooth = smooth_constrained_cyclic(path_cyclic)

number_of_steps = 200
"""
 - only proportional controller
 - robot will pass through target axis but with high overshoot/undershoot
"""
robot = make_robot(False)
x_p, y_p = p_controller(robot,
                        0.5,
                        number_of_steps)
"""
 - proportional and differential (pd) controller
 - robot will come to target axis with low overshoot/undershoot
"""
robot = make_robot(False)
x_pd, y_pd = pd_controller(robot,
                           0.3,
                           1.0,
                           number_of_steps)
"""
 - pd + systematic bias (in this case steering drift)
 - the cross track error (cte) will keep oscillating around initial value
 - instead of coming to target axis, because of steering drift
"""
robot = make_robot(True)
x_pd_drift, y_pd_drift = pd_controller(robot,
                                       0.3,
                                       3.0,
                                       number_of_steps)
"""
 - to reduce systematic bias like steering drift
 - we need to incorporate integral controller 
 - which subtracts systematic cte
 - and the value approaches to target without oscillating on the cte
"""
params = [0.3, 3.0, 0.01]
robot = make_robot(True)
x_pid, y_pid, err_pid = pid_controller(robot,
                                       0,
                                       params,
                                       number_of_steps)
"""
twiddle --> Coordinate Ascent Method

 - to further reduce the oscillation
 - we first optimize the params and then use the pid_controller
 - twiddle uses pid_controller inside it to optimize the parameter

 - in this case, we actually use pid, but instead of assignment value to params ourselves
 - we use twiddle to provide optimal params. after that we just use pid with these params.

 - so twiddle is not replacement for pid, instead it provides optimized params to be used
   for our specific pid controller
"""
params = twiddle()
robot = make_robot(True)
x_pid_twiddle, y_pid_twiddle, err_twiddle = pid_controller(robot,
                                                           0,
                                                           params,
                                                           number_of_steps)
robot = Robot(20.)
radius = 25.
orientation = np.pi / 2
robot.set(0.0, radius, orientation)
params_race_track = twiddle(pid_controller_race_track, radius)
# [10.0, 11.0, 0] --> can also be used instead of twiddle optimized params
x_pid_race_track, y_pid_race_track, err_race_track = pid_controller_race_track(robot,
                                                                               radius,
                                                                               params_race_track,
                                                                               number_of_steps)
"""
notice in the output that:
    - p controller keeps oscillating around target
    - pd controller reduces the oscillation around target
    - systematic bias (steering drift) makes pd controller to oscillate around the cross track error
    - pid controller removes the oscillation introduced by the systematic bias
    - tau_i determines how fast pid removes the oscillations and approaches to the target point
    - when pid is used with twiddle, it yields the best results and the oscillation is very low
    - twiddle optimizes 1 param at a time -- twiddle is good only in 1D
    - in real world, adaptive sampler is used in place of twiddle
"""
if PLOT_CONTROLLERS is True:
    plot_pid_controllers(x_p, y_p,  # p controller
                         x_pd, y_pd,  # pd controller
                         x_pd_drift, y_pd_drift,  # pd controller with drift
                         x_pid, y_pid,  # pid controller
                         x_pid_twiddle, y_pid_twiddle,
                         x_pid_race_track, y_pid_race_track)  # pid controller with twiddle
if PLOT_SMOOTHING is True:
    plot_smoothing(path,
                   path_smooth,
                   path_straight,
                   path_like_original)
if PLOT_CYCLIC_SMOOTHING is True:
    plot_cyclic_smoothing(path_cyclic,
                          path_cyclic_smooth)
if IS_PLOT_ENABLE is True:
    plt.show()
