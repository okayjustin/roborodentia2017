#!/usr/local/bin/python3

from helper_funcs import RunningStat
import numpy as np
from numpy import linalg
import serial, time, sys
from collections import deque
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
import pygame as pygame

MAX_PWM_CYCLES = 2047
BUTTON_PWM_CYCLES = 1700

# CALIBRATION VALUES, offset and scale
ACC_O_X = -482
ACC_O_Y = 48
ACC_O_Z = -972
ACC_S_X = 16522.56746066602
ACC_S_Y = 17033.517951277074
ACC_S_Z = 16921.85847403815
MAG_O_X = -76
MAG_O_Y = 382
MAG_O_Z = -78
MAG_S_X = 421.93019841175084
MAG_S_Y = 486
MAG_S_Z = 549.6501012927249
RANGE_O = 0.965025066
RANGE_S = -3.853474266

# Kalman settings
RANGE_VAR = 140
KAL_DT = 0.01
Q_VAR = 0.001  # Process covariance

class robot():
    def __init__(self):
        # pygame setup
        pygame.joystick.init()
        self.controller_connected = False
        if (pygame.joystick.get_count() == 1):
            pygame.init()
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(self.joystick.get_name())
            self.controller_connected = True
        self.joystickAxes = [0.0, 0.0, 0.0, 0.0]
        self.motorSpeeds = np.array([0, 0, 0, 0], dtype='f')     # speeds range from -1 to 1

        # robot setup
        self.max_hist_len = 500

        self.dt = RunningStat(self.max_hist_len)

        self.sensor_x = RunningStat(self.max_hist_len)
        self.sensor_y = RunningStat(self.max_hist_len)

        self.sensor_mag = RunningStat(self.max_hist_len)
        self.sensor_mag_ref = 0  # Stores the true compass home position angle
        self.sensor_mag_homed = 0    # Calculated angle relative to starting angle

        self.sensor_accel_x = RunningStat(self.max_hist_len)
        self.sensor_accel_y = RunningStat(self.max_hist_len)
        self.sensor_accel_z = RunningStat(self.max_hist_len)
        self.sensor_accel_offset_x = 0
        self.sensor_accel_offset_y = 0
        self.sensor_accel_offset_z = 0

        self.sensor_gyro_x = RunningStat(self.max_hist_len)
        self.sensor_gyro_y = RunningStat(self.max_hist_len)
        self.sensor_gyro_z = RunningStat(self.max_hist_len)
        self.sensor_gyro_offset_x = 0
        self.sensor_gyro_offset_y = 0
        self.sensor_gyro_offset_z = 0


        self.ser_available = False
        self.data_log_enable = False

        self.calibrating = True
        self.calibration_sample_count = 0

        # Kalman filter and states
        self.initKalman()
        self.kalman_x = RunningStat(self.max_hist_len)
        self.kalman_dx = RunningStat(self.max_hist_len)
        self.kalman_y = RunningStat(self.max_hist_len)
        self.kalman_dy = RunningStat(self.max_hist_len)

    def updateSensorValue(self):
        if (self.ser_available):
            try:
                while (self.ser.in_waiting > 0):
                    readback = self.ser.readline()
                    try:
                        readback_split = readback.decode().split(',')
                        if (len(readback_split) != 10):
                            print(readback)
                            return
                        dt = int(readback_split[0]) / 10.0  # Units of 0.1 ms, convert to ms
                        mag_val_raw = int (readback_split[1]) / 10.0 # Units of 0.1 degree, convert to degrees
                        rangeX_val_raw = int(readback_split[2]) * RANGE_O + RANGE_S # Units of mm
                        if (rangeX_val_raw > 1000):
                            rangeX_val_raw = 1000
                        rangeY_val_raw = int(readback_split[3]) * RANGE_O + RANGE_S # Units of mm
                        if (rangeY_val_raw > 1000):
                            rangeY_val_raw = 1000

                        accelX_val_raw = (int(readback_split[4]) - ACC_O_X) * 9806.65 / ACC_S_X # Units of mm/s/s)
                        accelY_val_raw = (int(readback_split[5]) - ACC_O_Y) * 9806.65 / ACC_S_Y # Units of mm/s/s)
                        accelZ_val_raw = (int(readback_split[6]) - ACC_O_Z) * 9806.65 / ACC_S_Z # Units of mm/s/s)

                        gyroX_val_raw = int(readback_split[7]) / 131.068 # Units of degrees per second
                        gyroY_val_raw = int(readback_split[8]) / 131.068 # Units of degrees per second
                        gyroZ_val_raw = int(readback_split[9]) / 131.068 # Units of degrees per second
                    except ValueError:
                        print("Readback error: ",end='')
                        print(readback)
                        return

                    # Log data
                    if (self.data_log_enable):
                        self.data_log += '%0.1f, %0.1f, %d, %d, \n' \
                                % (dt, mag_val_raw, rangeX_val_raw, rangeY_val_raw)

                    # Process time delta
                    self.dt.push(dt)

                    # Process magnetometer data
                    if (self.calibrating):
                        sensor_mag_homed = self.sensor_mag_ref - mag_val_raw
                    else:
                        sensor_mag_homed = self.loopAngle(self.sensor_mag_ref - mag_val_raw)
                    self.sensor_mag.push(sensor_mag_homed)

                    # Process rangefinders
                    self.sensor_x.push(self.limitValue(rangeX_val_raw, 0))
                    self.sensor_y.push(self.limitValue(rangeY_val_raw, 0))

                    # Process accelerometers
                    self.sensor_accel_x.push(round(accelX_val_raw - self.sensor_accel_offset_x))
                    self.sensor_accel_y.push(round(accelY_val_raw - self.sensor_accel_offset_y))
                    self.sensor_accel_z.push(round(accelZ_val_raw - self.sensor_accel_offset_z))

                    # Process gyros
                    self.sensor_gyro_x.push(gyroX_val_raw - self.sensor_gyro_offset_x)
                    self.sensor_gyro_y.push(gyroY_val_raw - self.sensor_gyro_offset_y)
                    self.sensor_gyro_z.push(gyroZ_val_raw - self.sensor_gyro_offset_z)

                    # Kalman filter
                    self.ukf.predict()
                    self.ukf.update([self.sensor_x.curVal(), self.sensor_y.curVal()])
                    self.kalman_x.push(self.limitValue(self.ukf.x[0], 0))
                    self.kalman_dx.push(self.ukf.x[1])
                    self.kalman_y.push(self.limitValue(self.ukf.x[2], 0))
                    self.kalman_dy.push(self.ukf.x[3])

                    # Collect many samples to get the average sensor value for offset calibration
                    if (self.calibrating):
                        if (self.calibration_sample_count == 0):
                            self.sensor_mag_ref = 0
                            self.sensor_accel_offset_x = 0
                            self.sensor_accel_offset_y = 0
                            self.sensor_accel_offset_z = 0
                            self.sensor_gyro_offset_x = 0
                            self.sensor_gyro_offset_y = 0
                            self.sensor_gyro_offset_z = 0

                        self.calibration_sample_count += 1
                        if (self.calibration_sample_count == self.max_hist_len + 1):
                            self.calibrating = False

                            # Use average angle measurement as the reference angle
                            self.sensor_mag_ref = self.loopAngle(-1 * self.sensor_mag.mean + 90.0)

                            # Use average accel measurement
                            self.sensor_accel_offset_x = self.sensor_accel_x.mean
                            self.sensor_accel_offset_y = self.sensor_accel_y.mean
                            self.sensor_accel_offset_z = self.sensor_accel_z.mean

                            # Use average gyro  measurement
                            self.sensor_gyro_offset_x = self.sensor_gyro_x.mean
                            self.sensor_gyro_offset_y = self.sensor_gyro_y.mean
                            self.sensor_gyro_offset_z = self.sensor_gyro_z.mean


                    # Process controller
                    if (self.controller_connected == True):
                        pygame.event.pump()
                        # Axis 0: Left stick, left -1.0, right 1.0
                        # Axis 1: Left stick, up -1.0, down 1.0
                        # Axis 2: Right stick, left -1.0, right 1.0
                        # Axis 3: Right stick, up -1.0, down 1.0
                        self.joystickAxes[0] = self.joystick.get_axis(0) * -1
                        self.joystickAxes[1] = self.joystick.get_axis(1) * -1
                        self.joystickAxes[2] = self.joystick.get_axis(2) * -1
                        self.joystickAxes[3] = self.joystick.get_axis(3) * -1
                        self.mechanumCommand(self.joystickAxes)

            except OSError:
                self.ser_available = False

    def loopAngle(self, angle):
        while ((angle >= 360.0) or (angle < 0.0)):
            if (angle >= 360.0): angle -= 360.0
            if (angle <    0.0): angle += 360.0
        return angle

    def mechanumCommand(self, joystickAxes):
        # Cartesian joystick vals to polar
        magnitude = np.sqrt(joystickAxes[0]**2 + joystickAxes[1]**2)
        angle = np.arctan2(joystickAxes[1], joystickAxes[0])
        rotation = joystickAxes[2]
        self.motorSpeeds[0] = magnitude * np.sin(angle + np.pi / 4) - rotation
        self.motorSpeeds[1] = magnitude * np.cos(angle + 5 * np.pi / 4) + rotation
        self.motorSpeeds[2] = magnitude * np.sin(angle + np.pi / 4) + rotation
        self.motorSpeeds[3] = magnitude * np.cos(angle + 5 * np.pi / 4) - rotation

        # Normalize speeds if any of them end up greater than 1
        max_speed = 0.0
        for speed in self.motorSpeeds:
            if (abs(speed) > max_speed):
                max_speed = abs(speed)
        if (max_speed > 1.0):
            self.motorSpeeds = self.motorSpeeds / max_speed
        print(self.motorSpeeds)

        # Send motor control commands
        for motorNum in range(0,4):
            if (self.motorSpeeds[motorNum] >= 0):
                self.botCmdMotor(motorNum, direction = 1, speed = int(MAX_PWM_CYCLES * self.motorSpeeds[motorNum]))
            else:
                self.botCmdMotor(motorNum, direction = -1, speed = int(MAX_PWM_CYCLES * (1 + self.motorSpeeds[motorNum])))

    def botCmdForwardButton(self):
        self.botCmdForward(BUTTON_PWM_CYCLES)

    def botCmdStopButton(self):
        self.botCmdStop()

    def botCmdReverseButton(self):
        self.botCmdReverse(MAX_PWM_CYCLES - BUTTON_PWM_CYCLES)

    def botCmdForward(self, pwmCycles):
        if (self.checkValue(pwmCycles, 0, MAX_PWM_CYCLES) != 0):
            raise ValueError("pwmCycles is not within 0 and %d" % MAX_PWM_CYCLES)
        for motorNum in range(0,4):
            self.botCmdMotor(motorNum, direction = 1, speed = pwmCycles)

    def botCmdStop(self):
        for motorNum in range(0,4):
            self.botCmdMotor(motorNum, direction = 1, speed = 0)

    def botCmdReverse(self, pwmCycles):
        if (self.checkValue(pwmCycles, 0, MAX_PWM_CYCLES) != 0):
            raise ValueError("pwmCycles is not within 0 and %d" % MAX_PWM_CYCLES)
        for motorNum in range(0,4):
            self.botCmdMotor(motorNum, direction = -1, speed = pwmCycles)

    def botCmdRotate(self, direction, pwmCycles):
        if (self.checkValue(pwmCycles, 0, MAX_PWM_CYCLES) != 0):
            raise ValueError("pwmCycles is not within 0 and %d" % MAX_PWM_CYCLES)
        if (direction == 1):       # Counterclockwise (+theta)
            self.botCmdMotor(0, direction = -1, speed = MAX_PWM_CYCLES - pwmCycles)
            self.botCmdMotor(1, direction = 1, speed = pwmCycles)
            self.botCmdMotor(2, direction = 1, speed = pwmCycles)
            self.botCmdMotor(3, direction = -1, speed = MAX_PWM_CYCLES - pwmCycles)
        if (direction == -1):       # Clockwise (-theta)
            self.botCmdMotor(0, direction = 1, speed = pwmCycles)
            self.botCmdMotor(1, direction = -1, speed = MAX_PWM_CYCLES - pwmCycles)
            self.botCmdMotor(2, direction = -1, speed = MAX_PWM_CYCLES - pwmCycles)
            self.botCmdMotor(3, direction = 1, speed = pwmCycles)

    # motorNum: 0 front left, 1 front right, 2 back right, 3 back left
    # direction (optional): -1 reverse, 1 forward
    # speed (optional): 0 to 2047, duty cycle value out of 2047
    def botCmdMotor(self, motorNum, direction = 0, speed = -1):
        cmdSequence = []
        if (direction == 1):
            cmdSequence.append('M' + str(motorNum) + 'DF\r')
        elif (direction == -1):
            cmdSequence.append('M' + str(motorNum) + 'DR\r')
        if ((speed >= 0) and (speed <= MAX_PWM_CYCLES)):
            cmdSequence.append('M' + str(motorNum) + 'S' + str(speed) + '\r')
        self.writeSerialSequence(cmdSequence)

    def angleControlTest(self, desired_angle):
        hyst = 20
        min_set_pwm_cycles = 1000

        m = (MAX_PWM_CYCLES - min_set_pwm_cycles) / 180.0
        error = self.sensor_mag_homed - desired_angle
        if (error > 180.0):
            error -= 360.0
        elif (error < -180.0):
            error += 360.0

        if (error > hyst):
            self.botCmdRotate(-1, self.limitValue(error * m + min_set_pwm_cycles, 0, MAX_PWM_CYCLES))
        elif (error < -1 * hyst):
            self.botCmdRotate(1, self.limitValue(error * -1 * m + min_set_pwm_cycles, 0, MAX_PWM_CYCLES))
        else:
            self.botCmdStop()

    def openSerial(self):
        ports = ['/dev/tty.usbmodem1413', '/dev/tty.usbmodem1423']
        self.ser_available = False
        self.ser = serial.Serial()
        self.ser.baudrate = 921600
        self.ser.bytesize = serial.EIGHTBITS #number of bits per bytes
        self.ser.parity = serial.PARITY_NONE #set parity check: no parity
        self.ser.stopbits = serial.STOPBITS_ONE #number of stop bits
        self.ser.timeout = 1            #non-block read
        self.ser.xonxoff = False     #disable software flow control
        self.ser.rtscts = False     #disable hardware (RTS/CTS) flow control
        self.ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
        self.ser.writeTimeout = 1     #timeout for write

        for port in ports:
            self.ser.port = port
            try:
                self.ser.close()
                self.ser.open()
                self.ser.reset_input_buffer()
                self.ser_available = True
                break
            except serial.serialutil.SerialException:
                pass

    def closeSerial(self):
        self.ser.close()
        self.ser_available = False

    def writeSerialSequence(self, cmd_seq):
        if (self.ser_available):
            for cmd in cmd_seq:
                self.ser.write(cmd.encode('utf-8'))

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

