import numpy as np


class ConstantVelocityKalmanTracker:
    """
    Linear Kalman filter for 2-D target tracking.

    State:
        [x, y, vx, vy]

    Measurement:
        [x, y]
    """

    def __init__(self, dt, process_var=2.0, measurement_var=36.0):
        self.dt = float(dt)

        self.F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ])

        q = float(process_var)
        self.Q = q * np.array([
            [dt**4/4, 0.0,      dt**3/2, 0.0],
            [0.0,      dt**4/4, 0.0,      dt**3/2],
            [dt**3/2, 0.0,      dt**2,   0.0],
            [0.0,      dt**3/2, 0.0,      dt**2],
        ])

        self.R = float(measurement_var) * np.eye(2)
        self.x = np.zeros(4)
        self.P = 100.0 * np.eye(4)
        self.initialized = False

    def initialize(self, position_xy):
        z = np.asarray(position_xy, dtype=float)
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.initialized = True

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x.copy()

    def update(self, position_xy):
        z = np.asarray(position_xy, dtype=float)
        innovation = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ innovation
        self.P = (np.eye(4) - K @ self.H) @ self.P
        return self.x.copy()

    def step(self, position_xy):
        if not self.initialized:
            self.initialize(position_xy)
            return self.x.copy()
        self.predict()
        return self.update(position_xy)
