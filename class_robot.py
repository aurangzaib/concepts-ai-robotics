import random
import numpy as np


class Robot(object):
    # --------
    # init:
    # creates robot and initializes location/orientation to 0, 0, 0
    #

    def __init__(self, length=0.5):
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.measurement_noise = 0.0
        self.num_collisions = 0
        self.num_steps = 0
        self.steering_drift = 0.0
    # --------
    # set:
    #	sets a robot coordinate
    #

    def set(self, new_x, new_y, new_orientation):

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0 * np.pi)

    # --------
    # set_noise:
    #	sets the noise parameters
    #

    def set_noise(self, new_s_noise, new_d_noise, new_m_noise=0.0):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)

    # --------
    # check:
    #    checks of the robot pose collides with an obstacle, or
    # is too far outside the plane

    def check_collision(self, grid):
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 1:
                    dist = np.sqrt((self.x - float(i)) ** 2 +
                                (self.y - float(j)) ** 2)
                    if dist < 0.5:
                        self.num_collisions += 1
                        return False
        return True

    def check_goal(self, goal, threshold=1.0):
        dist = np.sqrt((float(goal[0]) - self.x) ** 2 + (float(goal[1]) - self.y) ** 2)
        return dist < threshold

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    # --------
    # move:
    #    steering = front wheel steering angle, limited by max_steering_angle
    #    distance = total distance driven, most be non-negative

    def move(self, grid, steering, distance,
             tolerance=0.001, max_steering_angle=np.pi / 4.0):

        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # make a new copy
        res = Robot()
        res.length = self.length
        res.steering_noise = self.steering_noise
        res.distance_noise = self.distance_noise
        res.measurement_noise = self.measurement_noise
        res.num_collisions = self.num_collisions
        res.num_steps = self.num_steps + 1

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # Execute motion
        turn = np.tan(steering2) * distance2 / res.length

        if abs(turn) < tolerance:

            # approximate by straight line motion

            res.x = self.x + (distance2 * np.cos(self.orientation))
            res.y = self.y + (distance2 * np.sin(self.orientation))
            res.orientation = (self.orientation + turn) % (2.0 * np.pi)

        else:

            # approximate bicycle model for motion

            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            res.orientation = (self.orientation + turn) % (2.0 * np.pi)
            res.x = cx + (np.sin(res.orientation) * radius)
            res.y = cy - (np.cos(res.orientation) * radius)

        # check for collision
        # res.check_collision(grid)

        return res

    def move_simple(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # make a new copy
        # res = Robot()
        # res.length = self.length
        # res.steering_noise = self.steering_noise
        # res.distance_noise = self.distance_noise
        # res.steering_drift = self.steering_drift

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # apply steering drift
        steering2 += self.steering_drift

        # Execute motion
        turn = np.tan(steering2) * distance2 / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    # --------
    # sense:
    #

    def sense(self):

        return [random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise)]

    # --------
    # measurement_prob
    #    computes the probability of a measurement
    #

    def measurement_prob(self, measurement):

        # compute errors
        error_x = measurement[0] - self.x
        error_y = measurement[1] - self.y

        # calculate Gaussian
        error = np.exp(- (error_x ** 2) / (self.measurement_noise ** 2) / 2.0) \
                / np.sqrt(2.0 * np.pi * (self.measurement_noise ** 2))
        error *= np.exp(- (error_y ** 2) / (self.measurement_noise ** 2) / 2.0) \
                 / np.sqrt(2.0 * np.pi * (self.measurement_noise ** 2))

        return error

    def cte(self, radius):
        """
        this is the method which is used to provide the cross track error for a race track.
        previously we were using y as a cte but in real world we have to use other parameters 
        for the cte instead of only y. here we now use radius as a parameter

        a rectangle shaped race track is under consideration when applying logic below
        """

        # case - 1:
        # robot is in 1st region where x is less than radius.
        # we find the error using distance formula
        if self.x < radius:
            cte = np.sqrt((self.x - radius) ** 2 + (self.y - radius) ** 2) - radius
        # case - 2:
        # when robot is outside of boundary i.e. outside of 3*radius
        # we find the error using distance formula but taking care of
        # shifting x. distance from y doesnt have any effect.
        elif self.x > 3 * radius:
            cte = np.sqrt((self.x - 3 * radius) ** 2 + (self.y - radius) ** 2) - radius
        # case - 3:
        # robot is in upper region from center (inside or outside of the boundary)
        # in this case, error is y - diameter
        # we subtract from diameter because is measure from origin and not from center
        # -ve --> within boundary and above center
        # +ve --> outside of the upper boundary
        elif self.y > radius:
            cte = self.y - 2 * radius
        # case - 4:
        # robot in lower region from center (inside or outside of the boundary)
        # in this case, error is origin - y
        # -ve --> within boundary and below center
        # +ve --> outside of the lower boundary
        else:
            cte = 0 - self.y
        return cte

    def __repr__(self):
        # return '[x=%.5f y=%.5f orient=%.5f]'  % (self.x, self.y, self.orientation)
        return '[%.5f, %.5f]' % (self.x, self.y)