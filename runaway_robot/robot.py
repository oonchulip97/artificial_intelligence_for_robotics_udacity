from math import *
import random

# helper function to map all angles onto [-pi, pi]
def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

class robot:

    def __init__(self, x = 0.0, y = 0.0, heading = 0.0, turning = 2*pi/10, distance = 1.0):
        """This function is called when you create a new robot. It sets some of
        the attributes of the robot, either to their default values or to the values
        specified when it is created."""
        self.x = x
        self.y = y
        self.heading = heading
        self.turning = turning
        self.distance = distance
        self.turning_noise    = 0.0
        self.distance_noise    = 0.0
        self.measurement_noise = 0.0


    def set_noise(self, new_t_noise, new_d_noise, new_m_noise):
        """This lets us change the noise parameters, which can be very
        helpful when using particle filters."""
        self.turning_noise    = float(new_t_noise)
        self.distance_noise    = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)


    def move(self, turning, distance, tolerance = 0.001, max_turning_angle = pi):
        """This function turns the robot and then moves it forward."""
        # apply noise, this doesn't change anything if turning_noise
        # and distance_noise are zero.
        turning = random.gauss(turning, self.turning_noise)
        distance = random.gauss(distance, self.distance_noise)

        # truncate to fit physical limitations
        turning = max(-max_turning_angle, turning)
        turning = min( max_turning_angle, turning)
        distance = max(0.0, distance)

        # Execute motion
        self.heading += turning
        self.heading = angle_trunc(self.heading)
        self.x += distance * cos(self.heading)
        self.y += distance * sin(self.heading)

    def move_in_circle(self):
        """This function is used to advance the runaway target bot."""
        self.move(self.turning, self.distance)

    def sense(self):
        """This function represents the robot sensing its location. When
        measurements are noisy, this will return a value that is close to,
        but not necessarily equal to, the robot's (x, y) position."""
        return (random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise))

    def compute_weight(self, measurement):
        """This function compute importance weight which is useful for particle
         filter."""
        error_x = measurement[0] - self.x
        error_y = measurement[1] - self.y

        weight = (1 / sqrt(2.0 * pi * (self.measurement_noise ** 2))) \
        * exp(-(error_x ** 2) / (2.0 * (self.measurement_noise ** 2)))
        weight *= (1 / sqrt(2.0 * pi * (self.measurement_noise ** 2))) \
        * exp(-(error_y ** 2) / (2.0 * (self.measurement_noise ** 2)))

        return weight

    def compute_crosstrack_error(self, center, radius):
        """This function compute error with reference to circular track."""
        center_x = center[0]
        center_y = center[1]

        error = sqrt((self.x - center_x)**2 + (self.y - center_y)**2) \
        - radius

        return error


    def __repr__(self):
        """This allows us to print a robot's position"""
        return '[%.5f, %.5f]'  % (self.x, self.y)
