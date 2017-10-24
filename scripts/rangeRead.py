#!/usr/local/bin/python3

#import filterpy
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import itertools, statistics, sys, time, math
#import timeit
import serial
import numpy as np
#from PyQt5.QtGui import *
import PyQt5.QtGui as QtGui
import PyQt5.QtCore as QtCore
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
#from PyQt5.QtCore import *
#from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
from collections import deque

LINK_PLOTS = False
AXIS_PLOT_SIZE = 400
MAX_HIST_LEN = 100
LEAK_FACTOR_RANGEFINDER = 0.01  # Set from 0 to <1 for leaky integrator
LEAK_FACTOR_MAG = 0.01    # Set from 0 to <1 for leaky integrator
MEDIAN_LENGTH = 10

class XYWidget(pg.GraphicsLayoutWidget):
    def __init__(self, parent = None):
        super().__init__(parent)

    def resizeEvent(self, resizeEvent):
        if self.closed:
            return
        if self.autoPixelRange:
            try:
                ideal_height = self.parent().height() - 34 - AXIS_PLOT_SIZE
                ideal_width = self.parent().width() - 49 - AXIS_PLOT_SIZE
                if (ideal_height > ideal_width):
                    self.range = QtCore.QRectF(0, 0, ideal_width, ideal_width)
                else:
                    self.range = QtCore.QRectF(0, 0, ideal_height, ideal_height)
            except:
                pass
        super().setRange(self.range, padding=0, lockAspect=True, disableAutoPixel=False)
        self.updateMatrix()

    def sizeHint(self):
        return QSize(10000, 10000)

class XWidget(pg.GraphicsLayoutWidget):
    def __init__(self, parent = None):
        super().__init__(parent)

    def resizeEvent(self, resizeEvent):
        if self.closed:
            return
        if self.autoPixelRange:
            try:
                ideal_height = self.parent().height() - 34 - AXIS_PLOT_SIZE
                ideal_width = self.parent().width() - 49 - AXIS_PLOT_SIZE
                if (ideal_height > ideal_width):
                    self.range = QtCore.QRectF(0, 0, ideal_width, self.size().height())
                else:
                    self.range = QtCore.QRectF(0, 0, ideal_height, self.size().height())
            except:
                pass
        super().setRange(self.range, padding=0, lockAspect=True, disableAutoPixel=False)
        self.updateMatrix()

    def sizeHint(self):
        return QSize(10000, 10000)


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
#        self.pgcanvas.ci.layout.setColumnMaximumWidth(0, AXIS_PLOT_SIZE)
#        self.pgcanvas.ci.layout.setRowMaximumHeight(1, AXIS_PLOT_SIZE)
        self.positionlabel = QtGui.QLabel()
        self.fpslabel = QtGui.QLabel()
        self.fpslabel.setFixedWidth(200)

        # Add widgets/layouts to the layouts
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
        self.plot_y = self.pgcanvas.addPlot(0,0,labels={'bottom':'Latest sample #','left':'Y distance(mm)'})
        self.plot_y.showGrid(1,1,255)
        if (LINK_PLOTS):
            self.plot_y.getAxis('left').setTickSpacing(100, 50)
            self.plot_y.setYLink(self.plot_xy)
        self.plot_y.invertX()
        self.plot_y.setXRange(0, MAX_HIST_LEN, padding=0)
        self.plot_y_raw = self.plot_y.plot(pen='y')

        # X range plot
        self.plot_x = self.pgcanvas.addPlot(1,1,labels={'left':'Latest sample #','bottom':'X distance(mm)'})
        self.plot_x.showGrid(1,1,255)
        if (LINK_PLOTS):
            self.plot_x.setXLink(self.plot_xy)
            self.plot_x.getAxis('bottom').setTickSpacing(100, 50)
        self.plot_x.invertY()
        self.plot_x.setYRange(0, MAX_HIST_LEN, padding=0)
        self.plot_x_raw = self.plot_x.plot(pen='y')
        self.plot_x_kalman = self.plot_x.plot(pen='r')

        # Histogram plot
        self.plot_hist = self.pgcanvas.addPlot(2,0,1,2,labels={'left':'Count','bottom':'Distance (mm)'}, symbol='o', symbolSize=5, symbolPen=(255,255,255,200), symbolBrush=(0,0,255,150))
        self.plot_hist.showGrid(1,1,255)
        self.plot_hist.getAxis('bottom').setTickSpacing(100, 50)
        self.plot_hist.setXRange(0, 1000, padding=0)
        self.plot_hist.setYRange(0, MAX_HIST_LEN, padding=0)
        self.plot_hist_data = self.plot_hist.plot( stepMode=True, fillLevel=0, brush=(0,0,255,150))

        # Position arrow
        self.abs_position_arrow = pg.ArrowItem(angle=0, tipAngle=45, headLen=15, tailLen=15, tailWidth=3, brush='y')
        self.abs_position_arrow.rotate(90)
        self.plot_xy.addItem(self.abs_position_arrow)

        #### Set Data  #####################
        self.x = np.linspace(0, MAX_HIST_LEN + 1, num = MAX_HIST_LEN)
        self.histbins = bins=np.linspace(0, 1000, 1000)
        self.fps = 0.
        self.lastupdate = time.time()
        self.sensor_x = deque(MAX_HIST_LEN * [0], MAX_HIST_LEN)
        self.sensor_x_kalman = deque(MAX_HIST_LEN * [0], MAX_HIST_LEN)
        self.sensor_x_median = 0
        self.sensor_y = deque(MAX_HIST_LEN * [0], MAX_HIST_LEN)
        self.sensor_y_median = 0
        self.sensor_mag = deque(MAX_HIST_LEN * [0], MAX_HIST_LEN)
        self.sensor_mag_ref = 0  # Stores the true compass home position angle
        self.sensor_mag_homed = 0    # Calculated angle relative to starting angle
        self.sensor_mag.appendleft(-1)  # Append -1 so we can track when to set the home angle
        self.arrow_angle = 0
        self.ser_available = False

        # create the Kalman filter
        P = np.diag([500., 49.])
        self.kf = self.pos_vel_filter(x=(0,0), R=2, P=P, Q=0.01, dt=0.3)

        #### Start  #####################
        self._update()

    def _update(self):
        if (self.ser_available):
            self.updateSensorValue()

            # Convert deques to lists for easier processing
            sensor_x_list = list(self.sensor_x)
            sensor_x_kalman_list = list(self.sensor_x_kalman)
            sensor_y_list = list(self.sensor_y)

            # Calculate some stuff
            y,x = np.histogram(sensor_x_list + sensor_y_list, self.histbins)
            self.sensor_x_median = statistics.median(sensor_x_list[:MEDIAN_LENGTH])
            self.sensor_y_median = statistics.median(sensor_y_list[:MEDIAN_LENGTH])
            self.sensor_mag_homed = self.sensor_mag[0] - self.sensor_mag_ref

            # Update plots
            self.plot_x_raw.setData(self.sensor_x, self.x)
            self.plot_x_kalman.setData(self.sensor_x_kalman, self.x)
            self.plot_y_raw.setData(self.sensor_y)
            self.plot_hist_data.setData(x,y)
            self.abs_position_arrow.setPos(self.sensor_x_median,self.sensor_y_median)
            self.setArrowAngle(self.sensor_mag_homed)
        else:
            # If no serial available, try to open a new one
            self.openSerial()

        now = time.time()
        dt = (now-self.lastupdate)
        if dt <= 0:
            dt = 0.000000000001
        fps2 = 1.0 / dt
        self.lastupdate = now
        self.fps = self.fps * 0.9 + fps2 * 0.1
        tx = 'Mean Frame Rate:  {fps:0.0f} FPS'.format(fps=self.fps )
        pos_str = 'Position: \n    X: %3d mm\n    Y: %3d mm\n    Angle: %3d deg\n    Raw angle: %3d deg\n    Ref angle: %3d deg' % (self.sensor_x_median, self.sensor_y_median, self.sensor_mag_homed, self.sensor_mag[0], self.sensor_mag_ref)
        self.positionlabel.setText(pos_str)
        self.fpslabel.setText(tx)
        QtCore.QTimer.singleShot(1, self._update)

    def updateSensorValue(self):
        if (self.ser_available):
            try:
                while (self.ser.in_waiting > 0):
                    readback = self.ser.readline()
                    readback_split = readback.decode().split(',')
                    try:
                        if (len(readback_split) != 2):
                            return
                        sensor_num = readback_split[0]
                        range_val_raw = int(readback_split[1])
                    except ValueError:
                        print(readback)
                        return
                    if (sensor_num == '0'):  # Rangefinder x
                        if (range_val_raw > 1000):
                            range_val_raw = 1000
                        filt_val = int(self.sensor_x[0] * LEAK_FACTOR_RANGEFINDER + range_val_raw * (1 - LEAK_FACTOR_RANGEFINDER))
                        self.kf.predict()
                        self.kf.update(filt_val)

                        self.sensor_x.appendleft(filt_val)
                        self.sensor_x_kalman.appendleft(self.kf.x[0])
                    elif (sensor_num == '1'):  # Rangefinder y
                        if (range_val_raw > 1000):
                            range_val_raw = 1000
                        filt_val = int(self.sensor_y[0] * LEAK_FACTOR_RANGEFINDER + range_val_raw * (1 - LEAK_FACTOR_RANGEFINDER))
                        self.sensor_y.appendleft(filt_val)
                    elif (sensor_num == '2'):  # Magnetometer
                        filt_val = int(self.sensor_mag[0] * LEAK_FACTOR_MAG + range_val_raw * (1 - LEAK_FACTOR_MAG))
                        self.sensor_mag.appendleft(filt_val)

                        # Use first angle measurement as the reference angle
                        if (self.sensor_mag[1] == -1):
                            self.sensor_mag_ref = self.sensor_mag[0]

            except OSError:
                self.ser_available = False

    def setArrowAngle(self, angle):
        self.abs_position_arrow.rotate(angle - self.arrow_angle)
        self.arrow_angle = angle

    def openSerial(self):
        ports = ['/dev/tty.usbmodem1413', '/dev/tty.usbmodem1423']
        self.ser_available = False
        self.ser = serial.Serial()
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS #number of bits per bytes
        self.ser.parity = serial.PARITY_NONE #set parity check: no parity
        self.ser.stopbits = serial.STOPBITS_ONE #number of stop bits
        self.ser.timeout = 1            #non-block read
        self.ser.xonxoff = False     #disable software flow control
        self.ser.rtscts = False     #disable hardware (RTS/CTS) flow control
        self.ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
        self.ser.writeTimeout = 2     #timeout for write

        for port in ports:
            self.ser.port = port
            try:
                self.ser.open()
                self.ser.reset_input_buffer()
                self.ser_available = True
                break
            except serial.serialutil.SerialException:
                pass

    def closeSerial(self):
        self.ser.close()
        self.ser_available = False

    def closeEvent(self, *args, **kwargs):
        super(QtGui.QMainWindow, self).closeEvent(*args, **kwargs)
        if (self.ser_available): self.closeSerial()

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
