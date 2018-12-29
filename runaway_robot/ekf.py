from math import cos, sin, pi
from matrix import matrix

def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

class ExtendedKalmanFilter:
    """ State matrix has 5 dimensions:
        [x coordinate]
        [y coordinate]
        [velocity]
        [heading]
        [angular velocity]
    """

    def __init__(self, x_matrix = matrix([[]]), P_matrix = matrix([[]]), time_interval = 1):
        self.x = x_matrix
        self.P = P_matrix
        self.dt = time_interval

    def set_x(self, x_coord, y_coord, velocity, heading, angular_vel):
        self.x = matrix([
                        [x_coord],
                        [y_coord],
                        [velocity],
                        [heading],
                        [angular_vel]
                        ])

    def set_P(self,covariance):
        self.P = matrix([
                        [covariance, 0., 0., 0., 0.],
                        [0., covariance, 0., 0., 0.],
                        [0., 0., covariance, 0., 0.],
                        [0., 0., 0., covariance, 0.],
                        [0., 0., 0., 0., covariance]
                        ])

    def set_dt(self, time_interval):
        self.dt = time_interval

    def update(self, measurement):

        # measurement matrix
        Z = matrix([
                    [measurement[0]],
                    [measurement[1]]
                    ])

        # measurement transition matrix
        H = matrix([
                    [1., 0., 0., 0., 0.],
                    [0., 1., 0., 0., 0.]
                    ])

        # measurement transition matrix in Jacobian
        H_J = matrix([
                    [1., 0., 0., 0., 0.],
                    [0., 1., 0., 0., 0.],
                    ])

        # measurement noise
        R = matrix([
                    [0.1, 0.],
                    [0., 0.1]
                    ])

        # identity matrix
        I = matrix([
                    [1., 0., 0., 0., 0.],
                    [0., 1., 0., 0., 0.],
                    [0., 0., 1., 0., 0.],
                    [0., 0., 0., 1., 0.],
                    [0., 0., 0., 0., 1.]
                    ])

        y = Z - (H * self.x) # calculate difference in measurement and prediction
        S = H_J * self.P * H_J.transpose() + R # calculate Kalman Gain
        K = self.P * H_J.transpose() * S.inverse() # calculate Kalman Gain
        self.x = self.x + (K * y) # update x
        self.P = (I - (K * H_J)) * self.P # update P

    def predict(self):

        x_coord = self.x.value[0][0]
        y_coord = self.x.value[1][0]
        v = self.x.value[2][0]
        angle = self.x.value[3][0]
        omega = self.x.value[4][0]
        dt = self.dt

        # process noise matrix
        G = matrix([
                    [0.5*dt**2, 0.],
                    [0.5*dt**2, 0.],
                    [0., 0.],
                    [0., 0.5*dt**2],
                    [0., 0.],
                    ])

        # Pprocess noise matrix
        a = matrix([
                    [0.],
                    [0.]
                    ])

        # control input matrix
        u = matrix([
                    [0.],
                    [0.],
                    [0.],
                    [0.],
                    [0.]
                    ])

        # state trasition matrix
        F = matrix([
                    [1., 0., dt*cos(angle+omega*dt), 0., 0.],
                    [0., 1., dt*sin(angle+omega*dt), 0., 0.],
                    [0., 0., 1., 0., 0.],
                    [0., 0., 0., 1., dt],
                    [0., 0., 0., 0., 1.]
                    ])

        # state transition matrix in Jacobian
        F_J = matrix([
                    [1., 0., dt*cos(angle+omega*dt), -v*dt*sin(angle+omega*dt), -v*dt*sin(angle+omega*dt)],
                    [0., 1., dt*sin(angle+omega*dt), v*dt*cos(angle+omega*dt),  v*dt*cos(angle+omega*dt)],
                    [0., 0., 1., 0., 0.],
                    [0., 0., 0., 1., dt],
                    [0., 0., 0., 0., 1.]
                    ])

        # process noise covariance matrix
        Q = matrix([
                    [0., 0., 0., 0., 0.],
                    [0., 0., 0., 0., 0.],
                    [0., 0., 0., 0., 0.],
                    [0., 0., 0., 0., 0.],
                    [0., 0., 0., 0., 0.]
                    ])

        self.x = (F * self.x) + u + G * a # predict x
        self.P = F_J * self.P * F_J.transpose() + Q # predict P
        self.x.value[3][0] = angle_trunc(self.x.value[3][0]) # ensure heading within [-pi,pi)

    def get_data(self):
        return self.x, self.P

    def get_coordinates(self):
        return self.x.value[0][0], self.x.value[1][0]
