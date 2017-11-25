import numpy as np


class KalmanFilter(object):
    @staticmethod
    def find_probability(mean, variance, random_variable):
        normalize_factor = 1.0 / np.sqrt(2 * np.pi * variance)
        expo_argument = (-1.0 / 2.0) * (np.square(random_variable - mean) / variance)
        expo_factor = np.exp(expo_argument)
        return expo_factor * normalize_factor

    @staticmethod
    def one_dim__kalman_measurement_update(mean1, variance1, mean2, variance2):
        """
        measurement or sense
        """
        new_mean = (mean1 * variance2 + mean2 * variance1) / (variance1 + variance2)  # weighted sum
        new_variance = 1.0 / ((1.0 / variance1) + (1.0 / variance2))
        return [new_mean, new_variance]

    @staticmethod
    def one_dim__kalman_prediction_update(mean1, variance1, motion_mean, motion_variance):
        """
        prediction or move
        """
        new_mean = mean1 + motion_mean
        new_variance = variance1 + motion_variance
        return [new_mean, new_variance]

    @staticmethod
    def one_dim__kalman():
        # initial condition, high variance --> uncertainty initially
        mean = 0.
        variance = 10000.
        # measurement and prediction means
        measurements, predictions = [5., 6., 7., 9., 10.], [1., 1., 2., 1., 1.]
        # measurement and prediction variances
        measurement_variance, prediction_variance = 4., 2.

        for measurement, prediction in zip(measurements, predictions):
            # sense
            [mean, variance] = KalmanFilter.one_dim__kalman_measurement_update(mean,
                                                                               variance,
                                                                               measurement,
                                                                               measurement_variance)
            # move
            [mean, variance] = KalmanFilter.one_dim__kalman_prediction_update(mean,
                                                                              variance,
                                                                              prediction,
                                                                              prediction_variance)

        # mean value always increases
        # uncertainty or confidence adjusts itself
        print("result:", [mean, variance])

    @staticmethod
    def multi_variant__kalman_filter(estimate_matrix,  # x
                                     uncertainty_covariance,  # P
                                     motion_vector,  # u
                                     state_transition_matrix,  # F
                                     measurement_function,  # H
                                     noise,  # R
                                     identity_matrix,  # I
                                     measurements  # z
                                     ):
        """
        sensor only sees position and not the velocity
        but using kalman filter, we infer the velocity
        using the info from position and then it makes
        predictions for the future position
        """
        for index in range(len(measurements)):
            # move -- prediction step
            """
            x' = F*x + u
            P'  = F*P*F_t
            """
            estimate_matrix = (state_transition_matrix * estimate_matrix) + motion_vector
            uncertainty_covariance = state_transition_matrix * uncertainty_covariance * state_transition_matrix.getT()
            # sense -- measurement step
            """
            y = z_t - H*x
            S = H * P * H_t + R
            K = P * H_t * inv(S)
            x' = x + K*y
            P' = (I - K*H) * P
            """
            z_t = np.matrix([measurements[index]])
            measurement = z_t.transpose() - (measurement_function * estimate_matrix)
            system_error = measurement_function * uncertainty_covariance * measurement_function.transpose() + noise
            kalman_gain = uncertainty_covariance * measurement_function.transpose() * system_error.getI()
            estimate_matrix = estimate_matrix + (kalman_gain * measurement)
            uncertainty_covariance = (identity_matrix - (kalman_gain * measurement_function)) * uncertainty_covariance
        """
        printing estimates and confidence of kalman filter
        """
        print('estimation (x): \n', estimate_matrix)  # x_pos, x_vel, y_pos, y_vel
        print('prediction (P): \n', uncertainty_covariance)  # uncertainties of x and y pos and vel
        print('kalman gain(K): \n', kalman_gain) # kalman gain
        print('system error(S): \n', system_error) # system error
        print()
        """
        kalman filter now has predicted:
            the velocities and positions
            the covariance of positions and velocities
        """
        return estimate_matrix, uncertainty_covariance


x = np.matrix('0.;0.')  # estimate matrix --> x_pos, y_vel
P = np.matrix('1000. 0.;0. 1000.')  # uncertainty covariance
u = np.matrix('0.; 0.')  # motion vector --> external force
F = np.matrix('1. 1. ; 0. 1.')  # state transition matrix
H = np.matrix('1. 0.')  # measurement function --> x_pos observable, x_vel not-observable
R = np.matrix('1.')  # measurement noise
I = np.matrix('1. 0. ; 0. 1.')  # identity matrix
z = [1, 2, 3]  # measurements

# KalmanFilter.multi_variant__kalman_filter(x, P, u, F, H, R, I, z)

# 4 dimensional space: x and y, x_dot and y_dot
dt = 0.1
initial_xy = [-8., 8.]
# initial state vector; both velocities are zero initially
x__4d = np.matrix([[initial_xy[0]],  # x pos
                   [initial_xy[1]],  # x vel
                   [0.],  # y pos
                   [0.]]  # y vel
                  )
# uncertainty covariance matrix; high certainty for positions
# low certainty for velocities
# diagonal elements represent uncertainties, other elements are zero always
P__4d = np.matrix('0 0 0 0; 0 0 0 0; 0 0 1000. 0; 0 0 0 1000.')
# motion vector (no external motion)
u__4d = np.matrix('0.; 0.; 0; 0')
# state transition matrix
F__4d = np.matrix([[1., 0, dt, 0], [0, 1., 0, dt], [0, 0, 1., 0], [0, 0, 0, 1.]])
# measurement matrix
# only x and y are observable
# both velocities are not observables
H__4d = np.matrix('1. 0 0 0; 0 1. 0 0')
# measurement noise
R__4d = np.matrix('0.1 0; 0 0.1')
# identity matrix
I__4d = np.matrix('1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1')
# measurements --> x and y positions
z__4d = [[0., 4.], [8., 0.], [16., -4.], [24., -8.]]

KalmanFilter.multi_variant__kalman_filter(x__4d, P__4d,
                                          u__4d, F__4d,
                                          H__4d, R__4d,
                                          I__4d, z__4d)
