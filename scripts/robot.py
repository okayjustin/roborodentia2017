#!/usr/local/bin/python3

from helper_funcs import *
import numpy as np
from numpy import linalg
import serial, time, sys
from collections import deque
#from filterpy.kalman import KalmanFilter
#from filterpy.common import Q_discrete_white_noise
#from filterpy.kalman import MerweScaledSigmaPoints
#from filterpy.kalman import UnscentedKalmanFilter as UKF
#import pygame as pygame
from console import *
from timeit import default_timer as timer
import time
import struct

MAX_PWM_CYCLES = 2047
BUTTON_PWM_CYCLES = 1700
NUM_SENSORS = 11

# CALIBRATION VALUES, offset and scale
RANGE_S = 0.965025066
RANGE_O = -3.853474266
MAG_S_X = 1# / 421.93019841175084
MAG_S_Y = 1# / 486
MAG_S_Z = 1# / 549.6501012927249
MAG_O_X = 0#-76 * -MAG_S_X
MAG_O_Y = 0#382 * -MAG_S_Y
MAG_O_Z = 0#-78 * -MAG_S_Z
ACC_S_X = 1 / 16522.56746066602
ACC_S_Y = 1 / 17033.517951277074
ACC_S_Z = 1 / 16921.85847403815
ACC_O_X = -482 * -ACC_S_X
ACC_O_Y = 48   * -ACC_S_Y
ACC_O_Z = -972 * -ACC_S_Z
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

class Robot():
    data_cmd = 'B\n'.encode('utf-8')

    def __init__(self):
        self.motorSpeeds = np.array([0, 0, 0, 0], dtype='f')     # speeds range from -1 to 1

        # robot setup
        self.max_hist_len = 10

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
                        [RunningStat(self.max_hist_len), ACC_S_Z, ACC_O_Z], # ACCEL Z  9
#                        [RunningStat(self.max_hist_len), GYR_S_X, GYR_O_X, 0.], # GYRO X  10
#                        [RunningStat(self.max_hist_len), GYR_S_Y, GYR_O_Y, 0.], # GYRO Y  11
#                        [RunningStat(self.max_hist_len), GYR_S_Z, GYR_O_Z, 0.], # GYRO Z  12
                        [RunningStat(self.max_hist_len), 1.0, 0., 0.]] # MAG ORIENT        13

        self.console = SerialConsole()
        self.data_log_enable = False

        # Kalman filter and states
#        self.initKalman()
#        self.kalman_x = RunningStat(self.max_hist_len)
#        self.kalman_dx = RunningStat(self.max_hist_len)
#        self.kalman_y = RunningStat(self.max_hist_len)
#        self.kalman_dy = RunningStat(self.max_hist_len)

    def close(self):
        self.console.close()

    def updateSensorValue(self):
        if (self.console.ser_available):
            try:
                # Request data
                self.console.ser.write(self.data_cmd)
                data = self.console.ser.readline()
                vals = []
                for i in range(0, (len(data)-1)/2):
                    val.append(int.from_bytes(data[i*2:i*2+2], byteorder='big', signed=True))

                for sensor in self.sensors:
                    val = int.from_bytes(data[i*2:i*2+2], byteorder='big', signed=True)
                    # Push data * scaling factor + offset - DC_calibration
                    sensor[0].push(val * sensor[1] + sensor[2])

                # Log data
                if (self.data_log_enable):
                    self.data_log += '%0.1f, %0.1f, %d, %d, \n' \
                            % (dt, mag_val_raw, rangeX_val_raw, rangeY_val_raw)

                # Process time delta
#                self.dt.push(dt)
                # Kalman filter
#                    self.ukf.predict()
#                    self.ukf.update([self.sensor_x.curVal(), self.sensor_y.curVal()])
#                    self.kalman_x.push(limitValue(self.ukf.x[0], 0))
#                    self.kalman_dx.push(self.ukf.x[1])
#                    self.kalman_y.push(limitValue(self.ukf.x[2], 0))
#                    self.kalman_dy.push(self.ukf.x[3])

                # Process magnetometer data
                #if (self.calibrating):
                #    sensor_mag_homed = self.sensor_mag_ref - mag_val_raw
                #else:
                #    sensor_mag_homed = self.loopAngle(self.sensor_mag_ref - mag_val_raw)
                #self.sensor_mag.push(sensor_mag_homed)

            except OSError:
                print("Error")

    def printSensorVals(self):
        print("Sensor values:")
        #for i in range(0,NUM_SENSORS):
        for i in range(4,10):
            print("%+03.3f" % (self.sensors[i][0].curVal()))

    def calcHeading(self):
        pitch = np.arctan2(-self.sensors[7][0].curVal(), np.hypot(self.sensors[8][0].curVal(), self.sensors[9][0].curVal()))
        roll = np.arctan2(-self.sensors[8][0].curVal(), self.sensors[9][0].curVal())
        #print("Pitch: %f Roll: %f" % (np.degrees(pitch), np.degrees(roll)))
        xh = self.sensors[4][0].curVal() * np.cos(pitch) + \
                self.sensors[6][0].curVal() * np.sin(pitch)
        yh = self.sensors[4][0].curVal() * np.sin(roll) * np.sin(pitch) + \
                self.sensors[5][0].curVal() * np.cos(roll) - \
                self.sensors[6][0].curVal() * np.sin(roll) * np.cos(pitch)
        heading = np.degrees(np.arctan2(yh, xh)) + 180.0
        print("Heading: %+0.3f" % heading)
        return heading

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
        self.motorSpeeds[0] = vd * np.sin(theta_d + np.pi/4) - v_theta
        self.motorSpeeds[1] = vd * np.cos(theta_d + np.pi/4) + v_theta
        self.motorSpeeds[2] = vd * np.sin(theta_d + np.pi/4) + v_theta
        self.motorSpeeds[3] = vd * np.cos(theta_d + np.pi/4) - v_theta

        # Normalize ratios to 1 if the maxval is > 1
        maxval = np.amax(np.absolute(v))
        v = v / maxval if (maxval > 1) else v

#        # Cartesian joystick vals to polar
#        magnitude = np.sqrt(joystickAxes[0]**2 + joystickAxes[1]**2)
#        angle = np.arctan2(joystickAxes[1], joystickAxes[0])
#        rotation = joystickAxes[2]
#        self.motorSpeeds[0] = magnitude * np.sin(angle + np.pi / 4) - rotation
#        self.motorSpeeds[1] = magnitude * np.cos(angle + 5 * np.pi / 4) + rotation
#        self.motorSpeeds[2] = magnitude * np.sin(angle + np.pi / 4) + rotation
#        self.motorSpeeds[3] = magnitude * np.cos(angle + 5 * np.pi / 4) - rotation
#
#        # Normalize speeds if any of them end up greater than 1
#        max_speed = 0.0
#        for speed in self.motorSpeeds:
#            if (abs(speed) > max_speed):
#                max_speed = abs(speed)
#        if (max_speed > 1.0):
#            self.motorSpeeds = self.motorSpeeds / max_speed
#        print(self.motorSpeeds)

        # Send motor control commands
        self.botCmdMotor(self.motorSpeeds * MAX_PWM_CYCLES)

    # speed (optional): 0 to 2047, duty cycle value out of 2047
    def botCmdMotor(self, motorCmd):
        for cmd in motorCmd:
            if (checkValue(cmd, -MAX_PWM_CYCLES, MAX_PWM_CYCLES)):
                raise ValueError("motorCmd is not within bounds")
        cmd = 'M ' + str(motorCmd[0]) + ' ' + str(motorCmd[1]) + ' ' \
            + str(motorCmd[2]) + ' ' + str(motorCmd[3])
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

