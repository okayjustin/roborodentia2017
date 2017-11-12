#!/usr/local/bin/python3

import numpy as np
from numpy import linalg
import serial, time, sys
from collections import deque
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

MAX_PWM_CYCLES = 2047
BUTTON_PWM_CYCLES = 2000

class robot():
    def __init__(self):
        self.max_hist_len = 500

        self.dt = deque(self.max_hist_len * [-1], self.max_hist_len)

        self.sensor_x = deque(self.max_hist_len * [0], self.max_hist_len)
        self.sensor_x_kalman = deque(self.max_hist_len * [0], self.max_hist_len)

        self.sensor_y = deque(self.max_hist_len * [0], self.max_hist_len)

        self.sensor_mag = deque(self.max_hist_len * [0], self.max_hist_len)
        self.sensor_mag_ref = 0  # Stores the true compass home position angle
        self.sensor_mag_homed = 0    # Calculated angle relative to starting angle
        self.sensor_mag.appendleft(-1)  # Append -1 so we can track when to set the home angle

        self.sensor_accel_x = deque(self.max_hist_len * [0], self.max_hist_len)
        self.sensor_accel_y = deque(self.max_hist_len * [0], self.max_hist_len)
        self.sensor_accel_z = deque(self.max_hist_len * [0], self.max_hist_len)

        self.sensor_gyro_x = deque(self.max_hist_len * [0], self.max_hist_len)
        self.sensor_gyro_y = deque(self.max_hist_len * [0], self.max_hist_len)
        self.sensor_gyro_z = deque(self.max_hist_len * [0], self.max_hist_len)

        self.ser_available = False
        self.data_log_enable = False

        # create the Kalman filter
        P = np.diag([500., 49.])
        self.kf = self.pos_vel_filter(x=(250,0), R=5, P=P, Q=0, dt=0.3)

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
                        rangeX_val_raw = int(readback_split[2])  # Units of mm
                        if (rangeX_val_raw > 1000):
                            rangeX_val_raw = 1000
                        rangeY_val_raw = int(readback_split[3])  # Units of mm
                        if (rangeY_val_raw > 1000):
                            rangeY_val_raw = 1000

                        accelX_val_raw = 1.0* int(readback_split[4]) / pow(2, 14) # Units of g (9.8m/s/s)
                        accelY_val_raw = 1.0* int(readback_split[5]) / pow(2, 14) # Units of g (9.8m/s/s)
                        accelZ_val_raw = 1.0* int(readback_split[6]) / pow(2, 14) # Units of g (9.8m/s/s)

                        gyroX_val_raw = int(readback_split[7]) #/ 133.74693878
                        gyroY_val_raw = int(readback_split[8]) #/ 133.74693878
                        gyroZ_val_raw = int(readback_split[9]) #/ 133.74693878
                    except ValueError:
                        print("Readback error: ",end='')
                        print(readback)
                        return

                    # Log data
                    if (self.data_log_enable):
                        self.data_log += '%0.1f, %0.1f, %d, %d, \n' \
                                % (dt, mag_val_raw, rangeX_val_raw, rangeY_val_raw)

                    # Process time delta
                    self.dt.appendleft(dt)

                    # Process magnetometer data
                    self.sensor_mag.appendleft(mag_val_raw)
                    self.sensor_mag_homed = self.sensor_mag_ref - self.sensor_mag[0]
                    if (self.sensor_mag_homed > 360.0):
                        self.sensor_mag_homed -= 360.0
                    if (self.sensor_mag_homed < 0):
                        self.sensor_mag_homed += 360.0

                    # Use first angle measurement as the reference angle
                    if (self.sensor_mag[1] == -1):
                        self.sensor_mag_ref = self.sensor_mag[0] + 90.0

                    # Process rangefinders
                    self.kf.predict()
                    self.kf.update(rangeX_val_raw)
                    self.sensor_x.appendleft(rangeX_val_raw)
                    self.sensor_x_kalman.appendleft(self.kf.x[0])
                    self.sensor_y.appendleft(rangeY_val_raw)

                    # Process accelerometers
                    self.sensor_accel_x.appendleft(accelX_val_raw)
                    self.sensor_accel_y.appendleft(accelY_val_raw)
                    self.sensor_accel_z.appendleft(accelZ_val_raw)

                    # Process gyros
                    self.sensor_gyro_x.appendleft(gyroX_val_raw)
                    self.sensor_gyro_y.appendleft(gyroY_val_raw)
                    self.sensor_gyro_z.appendleft(gyroZ_val_raw)

            except OSError:
                self.ser_available = False

    def imu_calibrate(x, y, z):
        H = numpy.array([x, y, z, -y**2, -z**2, numpy.ones([len(x), 1])])
        H = numpy.transpose(H)
        w = x**2

        (X, residues, rank, shape) = linalg.lstsq(H, w)

        OSx = X[0] / 2
        OSy = X[1] / (2 * X[3])
        OSz = X[2] / (2 * X[4])

        A = X[5] + OSx**2 + X[3] * OSy**2 + X[4] * OSz**2
        B = A / X[3]
        C = A / X[4]

        SCx = numpy.sqrt(A)
        SCy = numpy.sqrt(B)
        SCz = numpy.sqrt(C)

        return ([OSx, OSy, OSz], [SCx, SCy, SCz])

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

    def limitValue(self, value, min_val, max_val):
        if (value > max_val):
            return max_val
        if (value < min_val):
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

#if __name__ == "__main__":
#    acc_f = open("acc.txt", 'r')
#    acc_x = []
#    acc_y = []
#    acc_z = []
#    for line in acc_f:
#        reading = line.split()
#        acc_x.append(int(reading[0]))
#        acc_y.append(int(reading[1]))
#        acc_z.append(int(reading[2]))
#
#    (offsets, scale) = calibrate(numpy.array(acc_x), numpy.array(acc_y), numpy.array(acc_z))
#    print(offsets)
#    print(scale)








