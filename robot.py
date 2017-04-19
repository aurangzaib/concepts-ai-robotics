import random
import numpy as np


class Robot(object):
    def __init__(self, length=20.0):
        """
        Creates robot and initializes location/orientation to 0, 0, 0.
        """
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, x, y, orientation):
        """
        Sets a robot coordinate.
        """
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)

    def set_noise(self, steering_noise, distance_noise):
        """
        Sets the noise parameters.
        """
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    def move(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):
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
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)
