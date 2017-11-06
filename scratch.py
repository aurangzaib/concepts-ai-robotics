def move(move_point, p):
    q = [0 for col in range(len(p))]
    move_point = move_point % len(p)
    for index in range(len(p)):
        q[move_point] = p[index]
        move_point = (move_point + 1) % len(p)
    print("q: {}".format(q))


def move_overshoot(move_point, p, p_overshoot, p_undershoot, p_exact):
    q = [0 for col in range(len(p))]
    move_point = move_point % len(p)
    for index in range(len(p)):
        q[move_point] += p[index] * p_exact
        q[move_point] += p[(index + 1) % len(p)] * p_undershoot
        q[move_point] += p[(index - 1) % len(p)] * p_overshoot
        move_point = (move_point + 1) % len(p)
    print("q overshoot: {}".format(q))
    return q


def move_stochastic(move_point, p, p_move):
    q = [0 for col in range(len(p))]
    move_point = move_point % len(p)
    for index in range(len(p)):
        q_move = p[index] * p_move
        q_stay = p[move_point] * (1.0 - p_move)
        q[move_point] = q_move + q_stay
        move_point = (move_point + 1) % len(p)
        print("q stochastic: {}".format(q))
    return q


def move_nxn(moves, p, p_move):
    q = [0 for col in range(len(p[0])) for row in range(len(p))]
    move_row, move_col = moves[0] % len(p), moves[1] % len(p[0])
    for row in range(len(p)):
        for col in range(len(p[0])):
            q_stay = p[move_row % len(p)][move_col % len(p[0])] * (1. - p_move)
            q_move = p[row][col] * (p_move)
            q[move_row][move_col] = q_move + q_stay
            move_col = (move_col + 1) % len(p[0])
        move_row = (move_row + 1) % len(p)
    print("q nxn: {}".format(q))
    return q


def sense(p, sensor_data, world_data, p_hit):
    q = [None for col in range(len(p))]
    p_miss = 1. - p_hit
    normalizer = 0
    for index in range(len(range(p))):
        hit_condition = sensor_data is world_data[index]
        q[index] = p[index] * p_hit if hit_condition else p[index] * p_miss
    normalizer += sum(q)
    for index in range(len(q)):
        q[index] /= normalizer
    return q


def sense_nxn(p, sensor_data, world_data, p_hit):
    q = [[0 for col in range(len(p[0]))] for row in range(len(p))]
    normalizer = 0
    for row in range(len(p)):
        for col in range(len(p[0])):
            hit = sensor_data is world_data[row][col]
            q[row][col] = p[row][col] * p_hit if hit else p[row][col] * (1. - p_hit)
        normalizer += sum(q[row])

    for row in range(len(q)):
        for col in range(len(q[0])):
            q[row][col] /= normalizer
    return q


def localize(p, move_points, world_map, sensor_measurements):
    q = p[:]
    for index in range(len(p)):
        # sense
        q = sense(q, sensor_measurements[index], world_map, 0.8)
        # move
        q = move_stochastic(move_points[index], q, p_move=0.8)
    # localized q
    print("localized q: {}".format(q))


def localize_nxn(sensor_measurements, world_map, motions):
    p_init = 1.0 / float(len(world_map) * len(world_map[0]))
    q = [[p_init for col in range(len(world_map[0]))] for row in range(len(world_map))]
    for motion, measurement in zip(motions, sensor_measurements):
        q = sense_nxn(q, measurement, world_map, 0.8)
        q = move_nxn(motion, q, 0.8)
    return q


def open_path_smoothing(path, alpha=0.0, beta=0.1, tolerance=0.000001):
    from copy import deepcopy
    smooth_path = deepcopy(path)
    change = tolerance
    while change >= tolerance:
        change = 0
        for row in range(1, len(path) - 1):
            for col in range(len(path[0])):
                old_value = smooth_path[row][col]
                n_row, p_row = row + 1, row - 1

                a = path[row][col] - smooth_path[row][col]
                b = smooth_path[n_row][col] + smooth_path[p_row][col] - 2. * smooth_path[row][col]

                smooth_path[row][col] += alpha * a + beta * b
                change += smooth_path[row][col] - old_value
    return smooth_path


def constrained_cyclic_smooth(path, alpha, beta, fix_points, tolerance):
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
                    n_row, nn_row = (row + 1) % len(path), (row + 2) % len(path)
                    p_row, pp_row = (row - 1) % len(path), (row - 2) % len(path)

                    a = path[row][col] - smooth_path[row][col]
                    b = smooth_path[n_row][col] + smooth_path[p_row][col] - 2 * smooth_path[row][col]
                    c = 2 * smooth_path[p_row][col] - smooth_path[pp_row][col] - smooth_path[row][col]
                    d = 2 * smooth_path[n_row][col] - smooth_path[nn_row][col] - smooth_path[row][col]

                    smooth_path[row][col] += alpha * a + beta * b + gamma * c + gamma * d
        change = abs(smooth_path[row][col] - old_value)


def p_controller(robot, tau_p, n=100, speed=1.0):
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


def pd_controller(robot, tau_p, tau_d, n=100, speed=1.0):
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


def pid_controller(robot, params, n=100, speed=1.0):
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


def pid_controller_track(robot, radius, params, n, speed=1.0):
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
