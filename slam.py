from class_robot import Robot
from class_particle import particles
from class_plannning import plan
import matplotlib.pyplot as plt
import random


# --------
#
# run:  runs control program for the robot
#

def run(grid, goal, s_path, params, printflag=False, speed=0.1, timeout=1000):
    plot_data = []
    robot = Robot()
    robot.set(0., 0., 0.)
    robot.set_noise(steering_noise, distance_noise, measurement_noise)
    particle_filter = particles(robot.x, robot.y, robot.orientation, steering_noise, distance_noise, measurement_noise)

    cte = 0.0
    err = 0.0
    N = 0

    index = 0  # index into the path

    while not robot.check_goal(goal) and N < timeout:

        diff_cte = - cte

        # ----------------------------------------
        # compute the CTE

        # start with the present robot estimate
        estimate = particle_filter.get_position()
        if index < len(s_path):
            start_point = s_path[index]
            end_point = s_path[index + 1]

            _delta = [end_point[0] - start_point[0], end_point[1] - start_point[1]]
            _R = [estimate[0] - start_point[0], estimate[1] - start_point[1]]

            _u = (_R[0] * _delta[0] + _R[1] * _delta[1]) / (_delta[0] ** 2 + _delta[1] ** 2)
            cte = (_R[1] * _delta[0] - _R[0] * _delta[1]) / (_delta[0] ** 2 + _delta[1] ** 2)

            if _u > 1:
                index += 1

        diff_cte += cte

        steer = - params[0] * cte - params[1] * diff_cte

        robot = robot.move(grid, steer, speed)
        particle_filter.move(grid, steer, speed)

        Z = robot.sense()
        particle_filter.sense(Z)

        if not robot.check_collision(grid):
            print '##### Collision ####'

        err += (cte ** 2)
        N += 1

        if printflag:
            print robot, cte, index, u

        plot_data.append([robot.x, robot.y])
    return [robot.check_goal(goal), robot.num_collisions, robot.num_steps, plot_data]


# ------------------------------------------------
#
# this is our main routine
#

def main(grid,
         init,
         goal,
         steering_noise,
         distance_noise,
         measurement_noise,
         weight_data,
         weight_smooth,
         p_gain,
         d_gain):
    path = plan(grid, init, goal)
    path.astar(goal)
    path.smooth(weight_data, weight_smooth)
    return run(grid, goal, path.s_path, [p_gain, d_gain])


# ------------------------------------------------
#
# input data and parameters
#


# grid format:
#   0 = navigable space
#   1 = occupied space

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 1, 1, 0],
        [0, 1, 0, 1, 0, 0],
        [0, 0, 0, 1, 0, 1],
        [0, 1, 0, 1, 0, 0]]

init = [0, 0]
goal = [len(grid) - 1, len(grid[0]) - 1]

steering_noise = 0.1
distance_noise = 0.03
measurement_noise = 0.3

weight_data = 0.1
weight_smooth = 0.2
p_gain = 2.0
d_gain = 6.0

plots_data = []
p_fig, p_path = plt.subplots()
p_path.set_title('path traces')

for index in range(0, 10):
    [goal_reached, collisions, steps, path] = main(grid,
                                                   init,
                                                   goal,
                                                   steering_noise,
                                                   distance_noise,
                                                   measurement_noise,
                                                   weight_data,
                                                   weight_smooth,
                                                   p_gain,
                                                   d_gain)
    plots_data.append(path)
for plot_data in plots_data:
    for data in plot_data:
        sample_index = random.randint(0, 1000)
        p_path.scatter(data[0], data[1], c=[[sample_index / 1000.,
                                             (1000 - sample_index) / 1000.,
                                             (1000 - sample_index) / 1000.]])
plt.show()


def twiddle(init_params):
    n_params = len(init_params)
    dparams = [1.0 for row in range(n_params)]
    params = [0.0 for row in range(n_params)]
    K = 10

    for i in range(n_params):
        params[i] = init_params[i]

    best_error = 0.0
    for k in range(K):
        ret = main(grid, init, goal,
                   steering_noise, distance_noise, measurement_noise,
                   params[0], params[1], params[2], params[3])
        if ret[0]:
            best_error += ret[1] * 100 + ret[2]
        else:
            best_error += 99999
    best_error = float(best_error) / float(k + 1)
    print best_error

    n = 0
    while sum(dparams) > 0.0000001:
        for i in range(len(params)):
            params[i] += dparams[i]
            err = 0
            for k in range(K):
                ret = main(grid, init, goal,
                           steering_noise, distance_noise, measurement_noise,
                           params[0], params[1], params[2], params[3], best_error)
                if ret[0]:
                    err += ret[1] * 100 + ret[2]
                else:
                    err += 99999
            print float(err) / float(k + 1)
            if err < best_error:
                best_error = float(err) / float(k + 1)
                dparams[i] *= 1.1
            else:
                params[i] -= 2.0 * dparams[i]
                err = 0
                for k in range(K):
                    ret = main(grid, init, goal,
                               steering_noise, distance_noise, measurement_noise,
                               params[0], params[1], params[2], params[3], best_error)
                    if ret[0]:
                        err += ret[1] * 100 + ret[2]
                    else:
                        err += 99999
                print float(err) / float(k + 1)
                if err < best_error:
                    best_error = float(err) / float(k + 1)
                    dparams[i] *= 1.1
                else:
                    params[i] += dparams[i]
                    dparams[i] *= 0.5
        n += 1
        print 'Twiddle #', n, params, ' -> ', best_error
    print ' '
    return params

# twiddle([weight_data, weight_smooth, p_gain, d_gain])
