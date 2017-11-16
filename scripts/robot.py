#!/usr/local/bin/python3

from helper_funcs import RunningStat
import numpy as np
from numpy import linalg
import serial, time, sys
from collections import deque
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

MAX_PWM_CYCLES = 2047
BUTTON_PWM_CYCLES = 2000

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

class robot():
    def __init__(self):
        self.max_hist_len = 500

        self.dt = RunningStat(self.max_hist_len)

        self.sensor_x = RunningStat(self.max_hist_len)
        self.sensor_x_kalman = RunningStat(self.max_hist_len)

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

        self.velocity_x = RunningStat(self.max_hist_len)

        self.ser_available = False
        self.data_log_enable = False

        self.calibrating = True
        self.calibration_sample_count = 0

        # create the Kalman filter
        P = np.diag([500., 50.])
        self.kf = self.pos_vel_filter(x=(1000,0), R=5, P=P, Q=0.1, dt=0.01)

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

                        accelX_val_raw = (int(readback_split[4]) - ACC_O_X) / ACC_S_X # Units of g (9.8m/s/s)
                        accelY_val_raw = (int(readback_split[5]) - ACC_O_Y) / ACC_S_Y # Units of g (9.8m/s/s)
                        accelZ_val_raw = (int(readback_split[6]) - ACC_O_Z) / ACC_S_Z # Units of g (9.8m/s/s)

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
                    self.kf.predict()
                    self.kf.update(self.limitValue(rangeX_val_raw, 0))
                    self.sensor_x.push(self.limitValue(rangeX_val_raw, 0))
                    self.sensor_x_kalman.push(self.limitValue(self.kf.x[0], 0))
                    self.velocity_x.push(self.kf.x[1])
                    self.sensor_y.push(self.limitValue(rangeY_val_raw, 0))

                    # Process accelerometers
                    self.sensor_accel_x.push(accelX_val_raw - self.sensor_accel_offset_x)
                    self.sensor_accel_y.push(accelY_val_raw - self.sensor_accel_offset_y)
                    self.sensor_accel_z.push(accelZ_val_raw - self.sensor_accel_offset_z)

                    # Process gyros
                    self.sensor_gyro_x.push(gyroX_val_raw - self.sensor_gyro_offset_x)
                    self.sensor_gyro_y.push(gyroY_val_raw - self.sensor_gyro_offset_y)
                    self.sensor_gyro_z.push(gyroZ_val_raw - self.sensor_gyro_offset_z)

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

            except OSError:
                self.ser_available = False

    def loopAngle(self, angle):
        while ((angle >= 360.0) or (angle < 0.0)):
            if (angle >= 360.0): angle -= 360.0
            if (angle <    0.0): angle += 360.0
        return angle

    def botCmdForwardButton(self):
        self.writeSerialSequence(['MLDF\r', 'MRDF\r', 'MLS%d\r' % BUTTON_PWM_CYCLES, 'MRS%d\r' % BUTTON_PWM_CYCLES])

    def botCmdStopButton(self):
        self.writeSerialSequence(['MLDF\r', 'MRDF\r','MLS0\r', 'MRS0\r'])

    def botCmdReverseButton(self):
        self.writeSerialSequence(['MLDR\r', 'MRDR\r', 'MLS%d\r' % (MAX_PWM_CYCLES - BUTTON_PWM_CYCLES), 'MRS%d\r' % (MAX_PWM_CYCLES - BUTTON_PWM_CYCLES)])

    def botCmdForward(self, pwmCycles):
        if (self.checkValue(pwmCycles, 0, MAX_PWM_CYCLES) != 0):
            raise ValueError("pwmCycles is not within 0 and %d" % MAX_PWM_CYCLES)
        self.writeSerialSequence(['MLDF\r', 'MRDF\r', 'MLS%d\r' % pwmCycles, 'MRS%d\r' % pwmCycles])

    def botCmdStop(self):
        self.writeSerialSequence(['MLDF\r', 'MRDF\r','MLS0\r', 'MRS0\r'])

    def botCmdReverse(self, pwmCycles):
        if (self.checkValue(pwmCycles, 0, MAX_PWM_CYCLES) != 0):
            raise ValueError("pwmCycles is not within 0 and %d" % MAX_PWM_CYCLES)
        self.writeSerialSequence(['MLDR\r', 'MRDR\r', 'MLS%d\r' % (MAX_PWM_CYCLES - pwmCycles), 'MRS%d\r' % (MAX_PWM_CYCLES - pwmCycles)])

    def botCmdRotate(self, direction, pwmCycles):
        if (self.checkValue(pwmCycles, 0, MAX_PWM_CYCLES) != 0):
            raise ValueError("pwmCycles is not within 0 and %d" % MAX_PWM_CYCLES)
        if (direction == 1):       # Counterclockwise (+theta)
            self.writeSerialSequence(['MLDR\r', 'MRDF\r', 'MLS%d\r' % (MAX_PWM_CYCLES - pwmCycles), 'MRS%d\r' % pwmCycles])
        if (direction == -1):       # Clockwise (-theta)
            self.writeSerialSequence(['MLDF\r', 'MRDR\r', 'MLS%d\r' % pwmCycles, 'MRS%d\r' % (MAX_PWM_CYCLES - pwmCycles)])

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

    def limitValue(self, value, min_val = None, max_val = None):
        if ((max_val != None) and (value > max_val)):
            return max_val
        if ((min_val != None) and (value < min_val)):
            return min_val
        return value

    def checkValue(self, value, min_val, max_val):
        if (value > max_val):
            return 1
        if (value < min_val):
            return -1
        return 0

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

    def pos_vel_filter(self, x, P, R, Q=0., dt=1.0):
        """ Returns a KalmanFilter which implements a
        constant velocity model for a state [x dx].T
        """

        kf = KalmanFilter(dim_x=2, dim_z=1)
        kf.x = np.array([x[0], x[1]]) # location and velocity
        kf.F = np.array([[1., dt],
                         [0.,  1.]])  # state transition matrix
        kf.H = np.array([[1., 0]])    # Measurement function
        kf.R *= R                     # measurement uncertainty
        if np.isscalar(P):
            kf.P *= P                 # covariance matrix
        else:
            kf.P[:] = P               # [:] makes deep copy
        if np.isscalar(Q):
            kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=Q)
        else:
            kf.Q[:] = Q
        return kf

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

