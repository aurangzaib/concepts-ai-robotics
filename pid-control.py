from copy import deepcopy
from helper import *

path = [[0, 0], [0, 0.5], [0, 1], [0, 1.5], [0, 2], [0.5, 2],
        [1, 2], [1.5, 2], [2, 2], [2.5, 2], [3, 2], [3.5, 2],
        [4, 2], [4, 2.5], [4, 3], [4, 3.5], [4, 4]]

path_cyclic = [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [5, 0],
               [6, 0], [6, 1], [6, 2], [6, 3], [5, 3], [4, 3],
               [3, 3], [2, 3], [1, 3], [0, 3], [0, 2], [0, 1]]
fix_points = [1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0]


def smooth_gradient_descent(opath, alpha=0.0, beta=0.1, tolerance=0.000001):
    """
    loop over each path on (i,j) and apply gradient descent equations.
    each time update the difference and store in change variable.

    the value of weight_data determines the smoothness of the edges
    smaller the values the smoother the edges, tending towards straight line
    i.e. edges will have 0 angles for data_weight=0 i.e. straight line

    the larger the values the sharper the edges
    on data_weight=1 we will have 90 degree edges
    """
    smooth_path = deepcopy(path)
    change = tolerance
    while change >= tolerance:
        change = 0
        for row in range(1, len(path) - 1):
            for col in range(len(path[0])):
                # previous value and row movements
                old_value = smooth_path[row][col]
                n_row, p_row = row + 1, row - 1
                # smoothing equations
                a = path[row][col] - smooth_path[row][col]
                b = smooth_path[n_row][col] + smooth_path[p_row][col] - 2. * smooth_path[row][col]
                # apply smoothing and update change
                smooth_path[row][col] += alpha * a + beta * b
                change += smooth_path[row][col] - old_value
    return smooth_path


def smooth_constrained_cyclic(path, alpha=0.07, beta=0.1, tolerance=0.000001):
    from copy import deepcopy
    gamma = 0.5 * beta
    smooth_path = deepcopy(path)
    change = tolerance
    while change >= tolerance:
        change = 0
        for row in range(len(path)):
            if not fix_points[row]:
                for col in range(len(path[0])):
                    old_value = smooth_path[row][col]
                    # rows movements
                    n_row, p_row = (row + 1) % len(path), (row - 1) % len(path)
                    nn_row, pp_row = (row + 1) % len(path), (row - 2) % len(path)
                    # smoothing equations
                    a = path[row][col] - smooth_path[row][col]
                    b = smooth_path[n_row][col] + smooth_path[p_row][col] - 2 * smooth_path[row][col]
                    c = 2 * (smooth_path[p_row][col]) - smooth_path[pp_row][col] - smooth_path[row][col]
                    d = 2 * (smooth_path[n_row][col]) - smooth_path[nn_row][col] - smooth_path[row][col]
                    # smoothing update
                    smooth_path[row][col] += (alpha * a) + (beta * b) + (gamma * c) + (gamma * d)
        change = abs(smooth_path[row][col] - old_value)
    return smooth_path


def p_controller(robot, tau_p, n=20, speed=1.0):
    # path tracing
    x_path, y_path = [], []
    for index in range(n):
        # save coordinates
        x_path.append(robot.x), y_path.append(robot.y)
        # update cte
        cte = robot.y
        # calculate p equations and steering
        p = -tau_p * cte
        steering = p
        # move robot
        robot.move_simple(steering, speed)
    return x_path, y_path


def pd_controller(robot, tau_p, tau_d, n=20, speed=1.0):
    # path tracing
    x_path, y_path = [], []
    # cross track errors
    cte = robot.y
    for index in range(n):
        # save coordinates
        x_path.append(robot.x), y_path.append(robot.y)
        # update cte
        cte_old = cte
        cte = robot.y
        # calculate pd equations and steering
        p = -tau_p * cte
        d = -tau_d * (cte - cte_old)
        steering = p + d
        # move robot
        robot.move_simple(steering, speed)
    return x_path, y_path


def pid_controller(robot, radius, params, n=100, speed=1.0):
    # pid params
    tau_p, tau_d, tau_i = params[0], params[1], params[2]
    # path tracing
    x_path, y_path, error = [], [], 0
    # cross track errors
    cte, cte_old, cte_sum = robot.y, robot.y, 0

    for index in range(2 * n):
        # save coordinates
        x_path.append(robot.x), y_path.append(robot.y)
        # update cte
        cte_old = cte
        cte_sum += cte
        cte = robot.y
        # calculate pid equations and steering
        p = -tau_p * cte
        d = -tau_d * (cte - cte_old)
        i = -tau_i * cte_sum
        steering = p + i + d
        # move robot
        robot.move_simple(steering, speed)
        # update error
        error += cte ** 2 if index >= n else 0
    return x_path, y_path, error / n


def pid_controller_race_track(robot, radius, params, n=100, speed=1.0):
    # pid params
    x_path, y_path, error, = [], [], 0
    # path tracing
    tau_p, tau_d, tau_i = params[0], params[1], params[2]
    # cross track errors
    cte, cte_sum = robot.cte(radius), 0

    for index in range(2 * n):
        # save coordinates
        x_path.append(robot.x), y_path.append(robot.y)
        # update cte
        cte_old = cte
        cte_sum += cte
        cte = robot.cte(radius)
        # calculate pid equations and steering
        p = -tau_p * cte
        d = -tau_d * (cte - cte_old)
        i = -tau_i * cte_sum
        steering = p + i + d
        # move robot
        robot.move_simple(steering, speed)
        # update error
        error += cte ** 2 if index >= n else 0
    return x_path, y_path, error / n


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
 - twiddle --> Coordinate Ascent Method

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
note in the output that:
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
