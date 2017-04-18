import numpy as np
from robot import Robot
import matplotlib.pyplot as plt

SHOULD_PRINT = False
PLOT_CONTROLLERS = True
PLOT_SMOOTHING = True
PLOT_CYCLIC_SMOOTHING = True
IS_PLOT_ENABLE = True


def make_robot(_with_systematic_bias=True):
    """
    Resets the robot back to the initial position and drift.
    You'll want to call this after you call `run`.
    """
    _robot = Robot()
    _robot.set(0, 1., 0)
    if _with_systematic_bias is True:
        _robot.set_steering_drift(10 / 180 * np.pi)
    return _robot


def print_paths(path, _new_path):
    for old, new in zip(path, _new_path):
        print '[' + ', '.join('%.3f' % x for x in old) + '] -> [' + ', '.join('%.3f' % x for x in new) + ']'
    print


def plot_smoothing(_path,
                   _path_smooth,
                   _path_straight,
                   _path_like_original):
    path_smooth_figure, path_smooth_plot = plt.subplots()
    path_straight_figure, path_straight_plot = plt.subplots()
    path_like_original_figure, path_like_original_plot = plt.subplots()
    old_fig, old_plot = plt.subplots()
    old_plot.set_title('old path')
    path_like_original_plot.set_title('path like original -- data=0.5 smooth=0')
    path_smooth_plot.set_title('path smooth -- data=0.1 smooth=0.1')
    path_straight_plot.set_title('path straight -- data=0 smooth=0.1')

    for _index in range(len(_path_smooth)):
        old_plot.scatter(_path[_index][0], _path[_index][1], c=244)
        path_like_original_plot.scatter(_path_like_original[_index][0], _path_like_original[_index][1], c=244)
        path_smooth_plot.scatter(_path_smooth[_index][0], _path_smooth[_index][1], c=244)
        path_straight_plot.scatter(_path_straight[_index][0], _path_straight[_index][1], c=244)


def plot_cyclic_smoothing(_path_cyclic,
                          _path_cyclic_smooth):
    path_cyclic_smooth_figure, path_cyclic_smooth_plot = plt.subplots()
    old_fig, old_plot = plt.subplots()
    old_plot.set_title('old path')
    path_cyclic_smooth_plot.set_title('smooth cyclic path')

    for _index in range(len(_path_cyclic)):
        old_plot.scatter(_path_cyclic[_index][0], _path_cyclic[_index][1], c=244)
        path_cyclic_smooth_plot.scatter(_path_cyclic_smooth[_index][0], _path_cyclic_smooth[_index][1], c=244)


def plot_pid_controllers(x_p, y_p,
                         x_pd, y_pd,
                         x_pd_drift, y_pd_drift,
                         x_pid, y_pid,
                         x_pid_twiddle, y_pid_twiddle,
                         x_pid_race_track, y_pid_race_track):
    p_fig, p_path = plt.subplots()
    pd_fig, pd_path = plt.subplots()
    pd_drift_fig, pd_drift_path = plt.subplots()
    pid_fig, pid_path = plt.subplots()
    pid_twiddle_fig, pid_twiddle_path = plt.subplots()
    pid_race_track_fig, pid_race_track_path = plt.subplots()

    p_path.set_title('p controller')
    pd_path.set_title('pd controller')
    pd_drift_path.set_title('pd controller with drift')
    pid_path.set_title('pid controller')
    pid_twiddle_path.set_title('pid twiddle controller')
    pid_race_track_path.set_title('pid race track')

    for _index in range(len(x_p)):
        p_path.scatter(x_p[_index], y_p[_index], c=244)
        pd_path.scatter(x_pd[_index], y_pd[_index], c=244)
        pd_drift_path.scatter(x_pd_drift[_index], y_pd_drift[_index], c=244)
        pid_path.scatter(x_pid[_index], y_pid[_index], c=244)
        pid_twiddle_path.scatter(x_pid_twiddle[_index], y_pid_twiddle[_index], c=244)
        pid_race_track_path.scatter(x_pid_race_track[_index], y_pid_race_track[_index], c=244)
