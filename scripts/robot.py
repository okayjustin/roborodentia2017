#!/usr/local/bin/python3

from helper_funcs import *
import numpy as np
from console import *
import struct
import time

#import time
#from numpy import linalg
#from filterpy.kalman import KalmanFilter
#from filterpy.common import Q_discrete_white_noise
#from filterpy.kalman import MerweScaledSigmaPoints
#from filterpy.kalman import UnscentedKalmanFilter as UKF
#import pygame as pygame
#from timeit import default_timer as timer

MAX_PWM_CYCLES = 2047
BUTTON_PWM_CYCLES = 1700

# CALIBRATION VALUES, offset and scale
RANGE_S = 0.965025066
RANGE_O = -3.853474266
MAG_S_X = 1 / 504.66782878550237
MAG_S_Y = 1 / 625.6254797589748
MAG_S_Z = 1 / 458.78374856276935
MAG_O_X = 55 * -MAG_S_X
MAG_O_Y = 249 * -MAG_S_Y
MAG_O_Z = 142 * -MAG_S_Z
ACC_S_X = 1 / 1030.9868342172633
ACC_S_Y = 1 / 1059.8985318881996
ACC_S_Z = 1 / 1063.2448503229036
ACC_O_X = 30 * -ACC_S_X
ACC_O_Y = 0 * -ACC_S_Y
ACC_O_Z = -76 * -ACC_S_Z
GYR_S_X = 1 / 131.068
GYR_S_Y = 1 / 131.068
GYR_S_Z = 1 / 131.068
GYR_O_X = 0.0
GYR_O_Y = 0.0
GYR_O_Z = 0.0

# Kalman settings
RANGE_VAR = 140
KAL_DT = 0.01
Q_VAR = 0.001  # Process covariance

X_OFFSET = 10
Y_OFFSET = 10

# Robot dimensions
LEN_X = 315.0      # length of the robot left to right
LEN_Y = 275.0      # length of the robot front to back
FIELD_XMAX = 2438.4 # Maximum x dimension in mm
FIELD_YMAX = 1219.2  # Maximum y dimension in mm

class Robot():
    sense_cmd = 'A\n'.encode('utf-8')
    data_cmd = 'B\n'.encode('utf-8')

    def __init__(self):
        self.motorSpeeds = np.array([0, 0, 0, 0], dtype='f')     # speeds range from -1 to 1

        # robot setup
        self.max_hist_len = 100
        self.dt = 0.05

        self.full_th = 0
        self.th_part = 0
        self.th_start = 0
        self.state = [RunningStat(3) for x in range(0,12)]

#        self.dt = RunningStat(self.max_hist_len)

        # Sensor array. Each contains a running history, offset factor, scaling factor
        # Rangefinders: Units of mm
        # Magnetometer: Units of  uTesla?. Arbitrary since converted to heading later
        # Accelerometer: Units of g's (gravity)
        # Gyro: Units of degrees per second
        self.sensors = [[RunningStat(self.max_hist_len), RANGE_S, RANGE_O], # RF 0     0
                        [RunningStat(self.max_hist_len), RANGE_S, RANGE_O], # RF 1     1
                        [RunningStat(self.max_hist_len), RANGE_S, RANGE_O], # RF 2     2
                        [RunningStat(self.max_hist_len), RANGE_S, RANGE_O], # RF 3     3
                        [RunningStat(self.max_hist_len), MAG_S_X, MAG_O_X], # MAG X    4
                        [RunningStat(self.max_hist_len), MAG_S_Y, MAG_O_Y], # MAG Y    5
                        [RunningStat(self.max_hist_len), MAG_S_Z, MAG_O_Z], # MAG Z    6
                        [RunningStat(self.max_hist_len), ACC_S_X, ACC_O_X], # ACCEL X  7
                        [RunningStat(self.max_hist_len), ACC_S_Y, ACC_O_Y], # ACCEL Y  8
                        [RunningStat(self.max_hist_len), ACC_S_Z, ACC_O_Z]] # ACCEL Z  9
        self.num_sensors = len(self.sensors)
#                        [RunningStat(self.max_hist_len), GYR_S_X, GYR_O_X, 0.], # GYRO X  10
#                        [RunningStat(self.max_hist_len), GYR_S_Y, GYR_O_Y, 0.], # GYRO Y  11
#                        [RunningStat(self.max_hist_len), GYR_S_Z, GYR_O_Z, 0.], # GYRO Z  12

        self.console = SerialConsole()
        self.data_log_enable = False

        # Kalman filter and states
#        self.initKalman()
#        self.kalman_x = RunningStat(self.max_hist_len)
#        self.kalman_dx = RunningStat(self.max_hist_len)
#        self.kalman_y = RunningStat(self.max_hist_len)
#        self.kalman_dy = RunningStat(self.max_hist_len)

    def openSerial(self):
        return self.console.openSerial()

    def close(self):
        self.console.close()
        try:
            self.angle_ann.close()
            self.transx_ann.close()
            self.transy_ann.close()
        except:
            pass

    def initializeNets(self):
        import tensorflow as tf
        from ann import ann
        # Load angle control ann
        print("Loading angle ANN")
        angle_model_path = './trained-models/models-angle/model.ckpt'
        self.angle_ann = ann(angle_model_path, state_dim = 3, action_dim = 1, action_space_high = 2.0)

        # Load transx control ann
        print("Loading transx ANN")
        transx_model_path = './trained-models/models-transx/model.ckpt'
        self.transx_ann = ann(transx_model_path, state_dim = 2, action_dim = 1, action_space_high = 2.0)

        # Load transy control ann
        print("Loading transy ANN")
        transy_model_path = './trained-models/models-transy/model.ckpt'
        self.transy_ann = ann(transy_model_path, state_dim = 2, action_dim = 1, action_space_high = 2.0)

    def predict(self):
        angle_obs = np.array([np.cos(self.state[4].curVal()), \
                np.sin(self.state[4].curVal()), self.state[5].curVal()])
        u_angle = -1 * self.angle_ann.predict(angle_obs)**5 / 70.

#        print("Th: %+0.3f | Thdot: %+0.3f | %+0.3f" % \
#                (np.degrees(self.state[4].curVal()), np.degrees(self.state[5].curVal()), u_angle))
        transx_obs = np.array([self.state[0].curVal() - self.state[6].curVal() + X_OFFSET, \
                self.state[1].winMean()])
        u_transx = self.transx_ann.predict(transx_obs)
        u_transx = 0

        transy_obs = np.array([self.state[1].curVal() - self.state[7].curVal() + Y_OFFSET, \
                self.state[3].winMean()])
        u_transy = self.transy_ann.predict(transy_obs)
        u_transy = 0

        u = np.array([u_angle, u_transx, u_transy])
        self.u = np.clip(u, -2., 2.)

    def execute(self):
        self.mechanumCommand(self.u[1], self.u[2], self.u[0])

    def updateSensorValue(self):
        if (self.console.ser_available):
            try:
                # Request data
                while True:
                    self.console.ser.reset_input_buffer()
                    self.console.ser.write(self.data_cmd)
                    data = self.console.ser.read(self.num_sensors * 2 + 1)
                    if (len(data) == self.num_sensors * 2 + 1):
                        if (data[-1] == 10):
                            break
                    else:
#                        print("Malformed UART data. Len: %d. Retrying..." % len(data))
                        pass

                # Start next sensor collection
                self.console.ser.write(self.sense_cmd)

                for i in range(0, self.num_sensors):
                    val = int.from_bytes(data[i*2:i*2+2], byteorder='big', signed=True)
                    # Push data * scaling factor + offset - DC_calibration
                    self.sensors[i][0].push(val * self.sensors[i][1] + self.sensors[i][2])

                # Log data
                if (self.data_log_enable):
                    self.data_log += '%0.1f, %0.1f, %d, %d, \n' \
                            % (dt, mag_val_raw, rangeX_val_raw, rangeY_val_raw)

                # Update state array-----------------------------------------------------

                # Update x and xdot states
                if (self.sensors[1][0].curVal() < self.sensors[3][0].curVal()):
                    x = self.sensors[1][0].curVal()
                else:
                    x = self.sensors[3][0].curVal()
                self.state[0].push(x)
                xdot = self.getVel(0)
                self.state[1].push(xdot)

                # Update y and ydot states
                y = (self.sensors[3][0].curVal() + FIELD_XMAX - self.sensors[0][0].curVal()) / 2.0
                self.state[2].push(y)
                ydot = self.getVel(2)
                self.state[3].push(ydot)

                # Update theta and thetadot states
                self.prev_th_part = self.th_part

                # Returns a heading from +pi to -pi
                # Tilt compensated heading calculation
                pitch = np.arcsin(-self.sensors[7][0].curVal())
                if (np.cos(pitch) == 0.):
                    pitch = 0.
                roll = np.arcsin(self.sensors[8][0].curVal() / np.cos(pitch))
#        print("Pitch: %f Roll: %f" % (np.degrees(pitch), np.degrees(roll)))
                xh = self.sensors[4][0].curVal() * np.cos(pitch) + \
                        self.sensors[6][0].curVal() * np.sin(pitch)
                yh = self.sensors[4][0].curVal() * np.sin(roll) * np.sin(pitch) + \
                        self.sensors[5][0].curVal() * np.cos(roll) - \
                        self.sensors[6][0].curVal() * np.sin(roll) * np.cos(pitch)
                self.th_part = np.arctan2(yh, xh)

                # If previous theta is near the upper limit (between 90 and 180 degrees) and
                # current theta is past the 180 degree limit (which means between -90 and -180 degrees)
                if (self.prev_th_part > np.pi/2 and self.th_part < -np.pi/2):
                    self.full_th += 1  # Increment full rotation count
                # If previous th is near the lower limit (between -90 and -180 degrees)
                elif (self.prev_th_part < -np.pi/2 and self.th_part > np.pi/2):
                    self.full_th -= 1  # Increment full rotation count

                th = self.full_th*2*np.pi + self.th_part - self.th_start
                self.state[4].push(th)
                thdot = self.getVel(4)
                self.state[5].push(thdot)

#                print("%+0.1f mean: %0.3f stddev: %+0.3f" % (np.degrees(self.th_stat.curVal()), \
#                        np.degrees(self.th_stat.winMean()), \
#                        np.degrees(self.th_stat.winStdDev())))

            except OSError:
                print("Error")

    # Calculates the average of forwards and backwards derivatives
    def getVel(self, sensorIdx):
        future = self.state[sensorIdx].vals[0]
        current = self.state[sensorIdx].vals[1]
        past = self.state[sensorIdx].vals[2]
        forward_deriv = future - current
        backwards_deriv = current - past
        return (forward_deriv + backwards_deriv)/(2*self.dt)


    # Recalculates the starting theta position to "zero out" theta
    def zeroTheta(self):
        # Reset DC offset
        self.th_start = 0

        # Get the theta averaged over a number of samples
        num_pts = 100
        #theta = []
        for i in range(0, num_pts):
            self.updateSensorValue()
        #    theta.append(self.state[4].curVal())
            time.sleep(0.05)
        self.th_start = self.state[4].mean() #np.sum(theta) / num_pts


    def printSensorVals(self):
        print("Sensor values:")
#        for i in range(0,self.num_sensors):
#            print("%+03.3f" % (self.sensors[i][0].curVal()), end = '| ')
#        print("")
#        return

        for i in range(4,10):
            print("%+03.3f, Var: %+0.3f" % (self.sensors[i][0].curVal(), self.sensors[i][0].winStdDev()))


    def loopAngle(self, angle):
        while ((angle >= 360.0) or (angle < 0.0)):
            if (angle >= 360.0): angle -= 360.0
            if (angle <    0.0): angle += 360.0
        return angle

    def mechanumCommand(self, x, y, th):
        x = np.clip(x, -2, 2)
        y = np.clip(y, -2, 2)
        th = np.clip(th, -2, 2)

        #  Trajectory control inputs
        v_theta = th
        vd = np.hypot(x, y) / 2.0
        theta_d = np.arctan2(y, x)

        # Calculate voltage ratios for each motor to acheive desired trajectory
        v = np.zeros(4)
        v[0] = vd * np.sin(theta_d + np.pi/4) - v_theta # Front left
        v[1] = vd * np.cos(theta_d + np.pi/4) - v_theta # Front right
        v[2] = vd * np.sin(theta_d + np.pi/4) + v_theta # Back right
        v[3] = vd * np.cos(theta_d + np.pi/4) + v_theta # Back left

        # Normalize ratios to 1 if the maxval is > 1
        maxval = np.amax(np.absolute(v))
        v = v / maxval if (maxval > 1) else v
        self.motorSpeeds = v

        # Send motor control commands
        self.cmdMotor([int(x * MAX_PWM_CYCLES) for x in self.motorSpeeds])

    # speed : 0 to 2047, duty cycle value out of 2047
    def cmdMotor(self, motorCmd):
        for cmd in motorCmd:
            if (checkValue(cmd, -MAX_PWM_CYCLES, MAX_PWM_CYCLES)):
                raise ValueError("motorCmd is not within bounds")
        cmd = 'M ' + str(motorCmd[0]) + ' ' + str(motorCmd[1]) + ' ' \
            + str(motorCmd[2]) + ' ' + str(motorCmd[3]) + '\n'
        self.console.writeSerialSequence([cmd])

    def initKalman(self):
        sigmas = MerweScaledSigmaPoints(4, alpha=.1, beta=2., kappa=1.)
        self.ukf = UKF(dim_x=4, dim_z=2, fx=self.f_cv, hx=self.h_cv, dt=KAL_DT, points=sigmas)
        self.ukf.x = np.array([0., 0., 0., 0.])      # Initial states: x, dx, y, dy
        self.ukf.P *= 1000                           # State covariance
        self.ukf.R = np.diag([RANGE_VAR, RANGE_VAR]) # Measurement covariance
        # Process covariance
        self.ukf.Q[0:2, 0:2] = Q_discrete_white_noise(dim=2, dt=KAL_DT, var=Q_VAR)
        self.ukf.Q[2:4, 2:4] = Q_discrete_white_noise(dim=2, dt=KAL_DT, var=Q_VAR)

    def f_cv(self, x, dt):
        """ state transition function for a
        constant velocity aircraft"""

        F = np.array([[1, dt, 0,  0],
                      [0,  1, 0,  0],
                      [0,  0, 1, dt],
                      [0,  0, 0,  1]])
        return np.dot(F, x)

    def h_cv(self, x):
        return np.array([x[0], x[2]])

    def startLog(self):
        self.data_log = 'dt (ms), Magnetometer (degrees), Rangefinder X (mm), Rangefinder Y (mm)\n'
        self.data_log_enable = True

    def stopLog(self):
        if (self.data_log_enable):
            self.data_log_enable = False
            timestr = time.strftime("%Y%m%d-%H%M%S")
            filename = 'datalog_' + timestr + '.csv'
            with open(filename, 'a') as datalog_file:
                datalog_file.write(self.data_log)

