import numpy as np


class KalmanFilter(object):
    @staticmethod
    def find_probability(mean, variance, random_variable):
        normalize_factor = 1.0 / np.sqrt(2 * np.pi * variance)
        expo_argument = (-1.0 / 2.0) * (np.square(random_variable - mean) / variance)
        expo_factor = np.exp(expo_argument)
        return expo_factor * normalize_factor

    @staticmethod
    def one_dim_kalman_measurement_update(mean1, variance1, mean2, variance2):
        new_mean = (mean1 * variance2 + mean2 * variance1) / (variance1 + variance2)  # weighted sum
        new_variance = 1.0 / ((1.0 / variance1) + (1.0 / variance2))
        return [new_mean, new_variance]

    @staticmethod
    def one_dim_kalman_prediction_update(mean1, variance1, motion_mean, motion_variance):
        new_mean = mean1 + motion_mean
        new_variance = variance1 + motion_variance
        return [new_mean, new_variance]

    @staticmethod
    def one_dim_kalman():
        # initial condition
        mean = 0.
        variance = 10000.  # very high uncertainty initially
        # mean measurement values
        measurements = [5., 6., 7., 9., 10.]
        # measurement variance
        measurement_variance = 4.
        # mean prediction values
        predictions = [1., 1., 2., 1., 1.]
        # prediction variance
        prediction_variance = 2.

        for index in range(len(measurements)):
            [mean, variance] = KalmanFilter.one_dim_kalman_measurement_update(mean,
                                                                              variance,
                                                                              measurements[index],
                                                                              measurement_variance)
            [mean, variance] = KalmanFilter.one_dim_kalman_prediction_update(mean,
                                                                             variance,
                                                                             predictions[index],
                                                                             prediction_variance)

        # mean value always increases
        # uncertainty or confidence adjusts itself
        print "result:", [mean, variance]
        """
        sensor only sees position and not the velocity
        but using kalman filter, we infer the velocity 
        using the info from position and then it makes
        predictions for the future position
        """

    @staticmethod
    def multi_variant__one_dimension_kalman_filter(estimate_matrix,  # x
                                                   uncertainty_covariance,  # P
                                                   motion_vector,  # u
                                                   state_transition_matrix,  # F
                                                   measurement_function,  # H
                                                   measurement_noise,  # R
                                                   identity_matrix,  # I
                                                   measurements  # z
                                                   ):
        for index in range(len(measurements)):
            # prediction step
            estimate_matrix = (state_transition_matrix * estimate_matrix) + motion_vector
            uncertainty_covariance = (
                                     state_transition_matrix * uncertainty_covariance) * state_transition_matrix.transpose()

            # measurement step
            z_t = np.matrix([measurements[index]])
            measurement = z_t.transpose() - (measurement_function * estimate_matrix)
            system_error = measurement_function * uncertainty_covariance * measurement_function.transpose() + measurement_noise
            kalman_gain = uncertainty_covariance * measurement_function.transpose() * system_error.getI()
            estimate_matrix = estimate_matrix + (kalman_gain * measurement)
            uncertainty_covariance = (identity_matrix - (kalman_gain * measurement_function)) * uncertainty_covariance

        print "estimation (x): \n", estimate_matrix
        print "prediction (P): \n", uncertainty_covariance


x = np.matrix('0.;0.')  # estimate matrix
P = np.matrix('1000. 0.;0. 1000.')  # uncertainty covariance
u = np.matrix('0.; 0.')  # motion vector
F = np.matrix('1. 1. ; 0. 1.')  # state transition matrix
H = np.matrix('1. 0.')  # measurement function
R = np.matrix('1.')  # measurement noise
I = np.matrix('1. 0. ; 0. 1.')  # identity matrix
z = [1, 2, 3]  # measurements

# KalmanFilter.multi_variant__one_dimension_kalman_filter(x, P,
#                                                         u, F,
#                                                         H, R,
#                                                         I, z)

# 4 dimensional space: x and y, x_dot and y_dot

dt = 0.1
initial_xy = [-4., 8.]
# initial state vector; both velocities are zero initially
x__2d = np.matrix([[initial_xy[0]], [initial_xy[1]], [0.], [0.]])
# uncertainty covariance matrix; high certainty for positions
# low certainty for velocities
# diagonal elements represent uncertainties, other elements are zero always
P__2d = np.matrix('0 0 0 0; 0 0 0 0; 0 0 1000. 0; 0 0 0 1000.')
# motion vector (no external motion)
u__2d = np.matrix('0.; 0.; 0; 0')
# state transition matrix
F__2d = np.matrix([[1., 0, dt, 0], [0, 1., 0, dt], [0, 0, 1., 0], [0, 0, 0, 1.]])
# measurement matrix
# only x and y are observable
# both velocities are not observables
H__2d = np.matrix('1. 0 0 0; 0 1. 0 0')
# measurement noise
R__2d = np.matrix('0.1 0; 0 0.1')
# identity matrix
I__2d = np.matrix('1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1')
# measurements
z__2d = [[1., 4.], [6., 0.], [11., -4.], [16., -8.]]

KalmanFilter.multi_variant__one_dimension_kalman_filter(x__2d, P__2d,
                                                        u__2d, F__2d,
                                                        H__2d, R__2d,
                                                        I__2d, z__2d)
