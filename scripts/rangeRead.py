#!/usr/local/bin/python3

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import itertools, statistics, sys, time, math
#import timeit
import serial
import numpy as np
import PyQt5.QtGui as QtGui
import PyQt5.QtCore as QtCore
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import pyqtgraph as pg
from collections import deque

AXIS_PLOT_SIZE = 400
MAX_HIST_LEN = 500
MEDIAN_LENGTH = 30
MAX_PWM_CYCLES = 2047
BUTTON_PWM_CYCLES = 2000

class App(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(App, self).__init__(parent)
        pg.setConfigOptions(useOpenGL=1)  # SPEEEEEED

        #### Create Gui Elements ###########
        self.mainbox = QtGui.QWidget()
        self.setCentralWidget(self.mainbox)
        self.mainbox.setLayout(QtGui.QBoxLayout(QBoxLayout.TopToBottom))
        self.hlayout = QHBoxLayout()
        self.vlayout = QVBoxLayout()

        # Define widgets
        self.pgcanvas = pg.GraphicsLayoutWidget()
        self.buttonLogStart = QtGui.QPushButton('Start data log')
        self.buttonLogStart.clicked.connect(self.startLog)
        self.buttonLogStop = QtGui.QPushButton('Stop data log')
        self.buttonLogStop.clicked.connect(self.stopLog)
        self.serialStatuslabel = QtGui.QLabel()
        self.serialStatuslabel.setText('Serial: not connected.')
        self.serialConsole = QtGui.QLineEdit()
        self.serialConsole.returnPressed.connect(self.writeSerialConsole)
        self.buttonForward = QtGui.QPushButton('Forward')
        self.buttonForward.clicked.connect(self.botCmdForwardButton)
        self.buttonStop = QtGui.QPushButton('Stop')
        self.buttonStop.clicked.connect(self.botCmdStopButton)
        self.buttonReverse = QtGui.QPushButton('Reverse')
        self.buttonReverse.clicked.connect(self.botCmdReverseButton)
        self.positionlabel = QtGui.QLabel()
        self.fpslabel = QtGui.QLabel()
        self.fpslabel.setFixedWidth(200)

        # Add widgets/layouts to the layouts
        self.vlayout.addWidget(self.buttonLogStart)
        self.vlayout.addWidget(self.buttonLogStop)
        self.vlayout.addWidget(self.serialStatuslabel)
        self.vlayout.addWidget(self.serialConsole)
        self.vlayout.addWidget(self.buttonForward)
        self.vlayout.addWidget(self.buttonStop)
        self.vlayout.addWidget(self.buttonReverse)
        self.vlayout.addWidget(self.positionlabel)
        self.vlayout.addWidget(self.fpslabel)
        self.hlayout.addWidget(self.pgcanvas)
        self.hlayout.addLayout(self.vlayout)
        self.mainbox.layout().addLayout(self.hlayout)

        # XY plot
        self.plot_xy = self.pgcanvas.addPlot(0,1,labels={'bottom':'X distance (mm)','left':'Y distance(mm)'})
#        self.plot_xy.setAspectLocked(1)
        self.plot_xy.showGrid(1,1,255)
        self.plot_xy.getAxis('left').setTickSpacing(100, 50)
        self.plot_xy.getAxis('bottom').setTickSpacing(100, 50)
        self.plot_xy.setXRange(0, 1000, padding=0)
        self.plot_xy.setYRange(0, 1000, padding=0)

        # Y range plot
        self.plot_y = self.pgcanvas.addPlot(0,0,labels={'left':'Latest sample #','bottom':'Y distance(mm)'})
        self.plot_y.showGrid(1,1,255)
        self.plot_y.invertY()
        self.plot_y.setYRange(0, MAX_HIST_LEN, padding=0)
        self.plot_y_raw = self.plot_y.plot(pen='y')
        self.plot_y_hist = self.plot_y.plot( stepMode=True, fillLevel=0, brush=(0,0,255,150))

        # X range plot
        self.plot_x = self.pgcanvas.addPlot(1,1,labels={'left':'Latest sample #','bottom':'X distance(mm)'})
        self.plot_x.showGrid(1,1,255)
        self.plot_x.invertY()
        self.plot_x.setYRange(0, MAX_HIST_LEN, padding=0)
        self.plot_x_raw = self.plot_x.plot(pen='y')
        self.plot_x_kalman = self.plot_x.plot(pen='r')
        self.plot_x_hist = self.plot_x.plot( stepMode=True, fillLevel=0, brush=(0,0,255,150))

        # Position arrow
        self.abs_position_arrow = pg.ArrowItem(angle=0, tipAngle=45, headLen=15, tailLen=15, tailWidth=3, brush='y')
        self.abs_position_arrow.rotate(90)
        self.plot_xy.addItem(self.abs_position_arrow)

        #### Set Data  #####################
        self.x = np.linspace(0, MAX_HIST_LEN + 1, num = MAX_HIST_LEN)
        self.histbins = bins=np.linspace(0, 1000, 1000)
        self.fps = 0.
        self.lastupdate = time.time()
        self.dt = deque(MAX_HIST_LEN * [-1], MAX_HIST_LEN)
        self.sensor_x = deque(MAX_HIST_LEN * [0], MAX_HIST_LEN)
        self.sensor_x_kalman = deque(MAX_HIST_LEN * [0], MAX_HIST_LEN)
        self.sensor_x_median = 0
        self.sensor_x_var = 0
        self.sensor_x_kal_var = 0
        self.sensor_y = deque(MAX_HIST_LEN * [0], MAX_HIST_LEN)
        self.sensor_y_median = 0
        self.sensor_y_var = 0
        self.sensor_mag = deque(MAX_HIST_LEN * [0], MAX_HIST_LEN)
        self.sensor_mag_ref = 0  # Stores the true compass home position angle
        self.sensor_mag_homed = 0    # Calculated angle relative to starting angle
        self.sensor_mag.appendleft(-1)  # Append -1 so we can track when to set the home angle
        self.arrow_angle = 0
        self.sensor_accel_x = deque(MAX_HIST_LEN * [0], MAX_HIST_LEN)
        self.sensor_accel_y = deque(MAX_HIST_LEN * [0], MAX_HIST_LEN)
        self.sensor_accel_z = deque(MAX_HIST_LEN * [0], MAX_HIST_LEN)

        # Control signals
        self.data_log_enable = False
        self.ser_available = False

        # create the Kalman filter
        P = np.diag([500., 49.])
        self.kf = self.pos_vel_filter(x=(250,0), R=5, P=P, Q=0, dt=0.3)

        #### Start  #####################
        self._update()

    # Gets the histogram but without empty bins
    def reducedHistogram(self, data_list, bins):
        y,x = np.histogram(data_list, bins)
        nonzero_indices = np.nonzero(y)
        y_reduced = y[nonzero_indices[0][0]:nonzero_indices[0][-1] + 1]
        x_reduced = x[nonzero_indices[0][0]:nonzero_indices[0][-1] + 2]
        return y_reduced, x_reduced

    def _update(self):
        if (self.ser_available):
            self.updateSensorValue()

            # Convert deques to lists for easier processing
            sensor_x_list = list(self.sensor_x)
            sensor_x_kalman_list = list(self.sensor_x_kalman)
            sensor_y_list = list(self.sensor_y)

            # Calculate some stuff
            plot_x_y,plot_x_x = self.reducedHistogram(sensor_x_list, self.histbins)
            plot_y_y,plot_y_x = self.reducedHistogram(sensor_y_list, self.histbins)
            self.sensor_x_median = statistics.median(sensor_x_kalman_list[:MEDIAN_LENGTH])
            self.sensor_x_var = np.var(sensor_x_list)
            self.sensor_x_kal_var = np.var(sensor_x_kalman_list)
            self.sensor_y_median = statistics.median(sensor_y_list[:MEDIAN_LENGTH])
            self.sensor_y_var = np.var(sensor_y_list)

            # Update plots
            self.plot_x_raw.setData(self.sensor_x, self.x)
            self.plot_x_kalman.setData(self.sensor_x_kalman, self.x)
            self.plot_y_raw.setData(self.sensor_y, self.x)
            self.plot_x_hist.setData(plot_x_x,plot_x_y)
            self.plot_y_hist.setData(plot_y_x,plot_y_y)
            self.abs_position_arrow.setPos(self.sensor_x_median,self.sensor_y_median)
            self.setArrowAngle(90.0 - self.sensor_mag_homed)
        else:
            # If no serial available, try to open a new one
            self.openSerial()

        # Update the labels on the side
        x_pos_str =         '    Median X: %3d mm\n' % self.sensor_x_median
        x_var_str =         '    Var X: %0.2f mm\n' % self.sensor_x_var
        x_kal_var_str =     '    Var Kal X: %0.2f mm\n' % self.sensor_x_kal_var
        x_var_ratio_str =   '    Var ratio X: %0.2f\n' % (self.sensor_x_kal_var / (self.sensor_x_var + 0.00000001))
        y_pos_str =         '    Median Y: %3d mm\n' % self.sensor_y_median
        y_var_str =         '    Var y: %0.2f mm\n' % self.sensor_y_var
        angle_str =         '    Angle: %0.1f deg\n' % (self.sensor_mag_homed)
        raw_angle_str =     '    Raw angle: %0.1f deg\n' % (self.sensor_mag[0])
        ref_angle_str =     '    Ref angle: %0.1f deg\n' % (self.sensor_mag_ref)
        x_accel_str =       '    X accel: %0.5f g\n' % self.sensor_accel_x[0]
        y_accel_str =       '    Y accel: %0.5f g\n' % self.sensor_accel_y[0]
        z_accel_str =       '    Z accel: %0.5f g\n' % self.sensor_accel_z[0]
        data_rate_str =     '    Data rate: %0.1f Hz\n' % (1000.0 / statistics.mean(self.dt))
        data_rate_per_str = '    Data rate per: %0.3f ms\n' % (statistics.mean(self.dt))
        data_rate_var_str = '    Data rate var: %0.4f ms\n' % (statistics.variance(self.dt))
        positionlabel_str = 'Data: \n' + x_pos_str + x_var_str + x_kal_var_str + x_var_ratio_str \
            + y_pos_str + y_var_str + angle_str + raw_angle_str + ref_angle_str + x_accel_str \
            + y_accel_str + z_accel_str+ data_rate_str + data_rate_per_str + data_rate_var_str
        self.positionlabel.setText(positionlabel_str)

        now = time.time()
        dt = (now-self.lastupdate)
        if dt <= 0:
            dt = 0.000000000001
        fps2 = 1.0 / dt
        self.lastupdate = now
        self.fps = self.fps * 0.9 + fps2 * 0.1
        self.fpslabel.setText('Mean Frame Rate:  {fps:0.0f} FPS'.format(fps=self.fps))
        QtCore.QTimer.singleShot(1, self._update)

    def updateSensorValue(self):
        if (self.ser_available):
            try:
                while (self.ser.in_waiting > 0):
                    readback = self.ser.readline()
                    try:
                        readback_split = readback.decode().split(',')
                        if (len(readback_split) != 7):
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
                    except ValueError:
                        print("Readback error: ",end='')
                        print(readback)
                        return

                    # Log data
                    if (self.data_log_enable):
                        self.data_log += '%0.1f, %0.1f, %d, %d\n' % (dt, mag_val_raw, rangeX_val_raw, rangeY_val_raw)

                    # Process time delta
                    self.dt.appendleft(dt)

                    # Process magnetometer data
                    self.sensor_mag.appendleft(mag_val_raw)
                    self.sensor_mag_homed = self.sensor_mag_ref - self.sensor_mag[0]
                    if (self.sensor_mag_homed > 360.0):
                        self.sensor_mag_homed -= 360.0
                    if (self.sensor_mag_homed < 0):
                        self.sensor_mag_homed += 360.0
                    self.angleControlTest(90.0)

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

            except OSError:
                self.closeSerial()

    def setArrowAngle(self, angle):
        self.abs_position_arrow.rotate(angle - self.arrow_angle)
        self.arrow_angle = angle

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
                self.serialStatuslabel.setText('Serial: connected.')
                break
            except serial.serialutil.SerialException:
                pass

    def writeSerialConsole(self):
        cmd_str = self.serialConsole.text()
        self.serialConsole.setText('')
        if (self.ser_available):
            cmd_str += '\r'
            self.ser.write(cmd_str.encode('utf-8'))

    def writeSerialSequence(self, cmd_seq):
        if (self.ser_available):
            for cmd in cmd_seq:
                self.ser.write(cmd.encode('utf-8'))

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

    def closeSerial(self):
        self.ser.close()
        self.ser_available = False
        self.serialStatuslabel.setText('Serial: not connected.')

    def closeEvent(self, *args, **kwargs):
        super(QtGui.QMainWindow, self).closeEvent(*args, **kwargs)
        if (self.ser_available): self.closeSerial()
        if (self.data_log_enable): self.stopLog()

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


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    thisapp = App()
    thisapp.show()
    sys.exit(app.exec_())
