

import numpy as np


class kalman_filter:
    
    # Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        
        self.P=P # state-estimate covariance
        self.Q=Q # process noise
        self.R=R # measurement noise
        self.x=x # state
        self.dt =dt

    def predict(self):

        # Jacobian of motion model
        self.A = self.jacobian_A()

        self.motion_model()

        # Jacobian of the measurement model
        self.C = self.jacobian_H()
        
        # Prediction advances the state and its uncertainty         
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # Calculate the Kalman gain and correct the estimates
    def update(self, z):
        
        # The Kalman gain wieghs the measurement correction
        kalman_gain= self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @ self.C.T + self.R)
        
        # Correct predicted state and covariance
        self.x= self.x + kalman_gain @ (z - self.measurement_model())
        self.P= (np.eye(6)-kalman_gain @ self.C) @ self.P
        
    # Predict measurements [v, w, ax, ay] from the state
    # For planar body-frame motion, ax = vdot and ay = v*w
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,# v
            w,# w
            vdot, # ax
            v * w, # ay
        ])

    # Integrate position, heading, and speed over dt.
    # Assume angular velocity and forward acceleration stay constant.
    def motion_model(self):
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        self.x = np.array([
            x + v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])
        
    # Jacobian of motion model
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0,              -v * np.sin(th) * dt, 0,          np.cos(th) * dt,  0],
            [0, 1,              v * np.cos(th) * dt, 0,          np.sin(th) * dt,  0],
            [0, 0,                1, dt,           0,  0],
            [0, 0,                0, 1,            0,  0],
            [0, 0,                0, 0,            1,  dt],
            [0, 0,                0, 0,            0,  1 ]
        ])
    
    # Jacobian of measurement model
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0], # v
            [0,0,0  , 1, 0, 0], # w
            [0,0,0  , 0, 0, 1], # ax
            [0,0,0  , v, w, 0], # ay
        ])
        
    def get_states(self):
        return self.x
