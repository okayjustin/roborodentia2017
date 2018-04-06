#!/usr/local/bin/python3
'''
Simulates sensors
'''

import numpy as np

class SimRangefinder():
    """ Simulates a rangefinder.
    x: x-position in robot coordinates
    y: y-position in robot coordinates
    theta: orientation in robot coordinates
    min_range: minimum measurement range
    max_range: maximum measurement range
    meas_period: amount of time between measurements
    """
    def __init__(self,x,y,theta,dt, field_xmax, field_ymax, max_range=1200.0,meas_period=0.033):
        self.type = 'Rangefinder'
        self.dt = dt
        self.field_xmax = field_xmax
        self.field_ymax = field_ymax
        self.theta = np.radians(theta)
        self.position_theta = np.arctan2(y,x)
        self.radius = np.hypot(x,y)
        self.max_range = max_range
        self.timebank = 0
        self.meas_period = meas_period
        self.meas = np.array([0.,0.])
        self.dim = [(0.,0.),(0.,0.)]
        self.high = np.array([1230., 1600.])
        self.low = np.array([0., -1600.])

    def update(self, state):
        """ Returns the measurement of the rangefinder in mm.
        robot: robot object
        """
        # Add some available time to the timebank
        self.timebank += self.dt
        # Only make a measurement if there's enough time in the bank
        if (self.timebank < self.meas_period):
            return
        # Use up some time from the timebank
        self.timebank -= self.meas_period

        # Calculate the sensor's position and orientation in the field
        # rf_field_theta is used to calculate where the sensor is but not where its oriented
        rf_field_theta = state[4] + self.position_theta
        x1 = state[0] + self.radius * np.cos(rf_field_theta)
        y1 = state[2] + self.radius * np.sin(rf_field_theta)

        # Calculate cos and sin of the angle of where the rangefinder is oriented
        rf_aim_theta = state[4]  + self.theta
        cosval = np.cos(rf_aim_theta)
        sinval = np.sin(rf_aim_theta)

        if cosval >= 0:
            dist_x = self.field_xmax - x1
        else:
            dist_x = -1 * x1
        if sinval >= 0:
            dist_y = self.field_ymax - y1
        else:
            dist_y = -1 * y1

        # Calculate the two possible measurements
        try:
            hx = dist_x / cosval # If the hypotenuse extends to a vertical wall
            hy = dist_y / sinval if sinval != 0 else dist_y / 0.00000001 # If the hypotenuse extends to a horizontal wall

            # The correct measurement is the shorter one
            if (hx < hy):
                dist = hx
                x2 = x1 + dist_x
                y2 = y1 + dist * sinval
            else:
                dist = hy
                x2 = x1 + dist * cosval
                y2 = y1 + dist_y
        except:
            if (cosval != 0):
                dist = dist_x / cosval
                x2 = x1 + dist_x
                y2 = y1 + dist * sinval
            else:
                dist = dist_y / sinval
                x2 = x1 + dist * cosval
                y2 = y1 + dist_y

        # Add noise to measurement
        #sigma = 2E-05 * pow(dist,2) - 0.0062 * dist + 2.435
        #dist += np.random.normal(0, sigma)

        # Only update if measurement is in range
        if ((dist > 30) and (dist < 1200)):
            self.dim = [(x1,y1),(x2,y2)]
            dist_dot = (dist - self.meas[0]) / self.dt
            self.meas = np.array([dist, dist_dot])
        else:
            if (dist <= 30):
                self.meas = np.array([0.0, 0.0])
            else:
                self.meas = np.array([1230.0, 0.0])


class SimIMU():
    def __init__(self,dt,meas_period = 0.001, sigma = 0.0):
        self.type = 'IMU'
        self.dt = dt
        self.timebank = 0
        self.prevth = 0
        self.meas = np.array([0., 0., 0.])
        self.meas_period = meas_period
        self.sigma = sigma
        self.high = np.array([1., 1., 8.])
        self.low = np.array([-1., -1., -8.])

    def update(self, state):
        # Add some available time to the timebank
        self.timebank += self.dt
        # Only make a measurement if there's enough time in the bank
        if (self.timebank < self.meas_period):
            return
        # Use up some time from the timebank
        self.timebank -= self.meas_period

        # Get angle measurement and estimate a velocity
             #(robot.theta + np.random.normal(0, self.sigma)) % 360.0
        th = state[4]
        thdot = (th - self.prevth) / self.dt
        self.prevth = th

        # Get x/y components of theta
        cos = np.cos(th)
        sin = np.sin(th)

        self.meas = np.array([cos, sin, thdot])

class SimDesiredXY():
    def __init__(self,direction, field_xmax, field_ymax, len_x, len_y):
        self.dir = direction
        self.type = 'DesiredXY'
        self.field_xmax = field_xmax
        self.field_ymax = field_ymax
        self.len_x = len_x
        self.len_y = len_y
        self.meas = np.array([0.])
        if direction == 'x':
            self.high = np.array([self.field_xmax])
            self.low = np.array([0])
        elif direction == 'y':
            self.high = np.array([self.field_ymax])
            self.low = np.array([0])
        else:
            raise ValueError("Bad value for direction")

    def update(self, state):
        if (self.dir == 'x'):
            self.meas = np.array([state[6]])
        else:
            self.meas = np.array([state[7]])

class SimActualXY():
    def __init__(self, direction, field_xmax, field_ymax, len_x, len_y):
        self.dir = direction
        self.type = 'ActualXY'
        self.field_xmax = field_xmax
        self.field_ymax = field_ymax
        self.len_x = len_x
        self.len_y = len_y
        self.meas = np.array([0., 0.])
        if direction == 'x':
            self.high = np.array([self.field_xmax, 1500.])
            self.low = np.array([0., -1500.])
        elif direction == 'y':
            self.high = np.array([self.field_ymax, 1500.])
            self.low = np.array([0., -1500.])
        else:
            raise ValueError("Bad value for direction")

    def update(self, state):
        if (self.dir == 'x'):
            self.meas = np.array([state[0] - state[6], state[1]]) # + 131.6
            #print(self.meas)
        else:
            self.meas = np.array([state[2], state[3]])


class SimMicroSW():
    def __init__(self):
        return
