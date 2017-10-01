#!/usr/local/bin/python3

from __future__ import print_function
import sys
import time
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

AXIS_PLOT_SIZE = 200
AXIS_PLOT_CORRECTION = 15
MAX_HIST_LEN = 100

class XYWidget(pg.GraphicsLayoutWidget):
    def __init__(self, parent = None):
        super().__init__(parent)

    def resizeEvent(self, resizeEvent):
        if self.closed:
            return
        if self.autoPixelRange:
            try:
                ideal_height = self.parent().height() - 234
                ideal_width = self.parent().width() - 249
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
                ideal_height = self.parent().height() - 234
                ideal_width = self.parent().width() - 249
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

        #### Create Gui Elements ###########
        self.mainbox = QtGui.QWidget()
        self.setCentralWidget(self.mainbox)

        # Define layouts
        self.mainbox.setLayout(QtGui.QBoxLayout(QBoxLayout.TopToBottom))
        self.toprow = QHBoxLayout()
        self.bottomrow = QHBoxLayout()

        # Define widgets
        self.canvas_x = XWidget()
        self.canvas_x.setAlignment(Qt.AlignLeft)
        self.canvas_x.setFixedHeight(AXIS_PLOT_SIZE)
        self.canvas_y = pg.GraphicsLayoutWidget()
        self.canvas_y.setFixedWidth(AXIS_PLOT_SIZE + AXIS_PLOT_CORRECTION)
        self.canvas_xy = XYWidget()
        self.canvas_xy.setAlignment(Qt.AlignLeft)
        self.label = QtGui.QLabel()
        self.label.setFixedWidth(AXIS_PLOT_SIZE + AXIS_PLOT_CORRECTION)

        # Add widgets/layouts to the layouts
        self.toprow.addWidget(self.canvas_y, 0)
        self.toprow.addWidget(self.canvas_xy, 1, Qt.AlignLeft)
        self.bottomrow.addWidget(self.label, 0, Qt.AlignCenter)
        self.bottomrow.addWidget(self.canvas_x, 1)
        self.mainbox.layout().addLayout(self.toprow, 0)
        self.mainbox.layout().addLayout(self.bottomrow, 1)

        # Add things to the widgets
        self.plot_x = self.canvas_x.addPlot()
        self.plot_y = self.canvas_y.addPlot()
        self.plot_xy = self.canvas_xy.addPlot(pen = None, symbol = 'o', setAspectLocked = True)
        self.plot_x.invertY()
        self.plot_x.setRange(yRange=[0, MAX_HIST_LEN])
        self.plot_x.setRange(xRange=[0, 1000], yRange=[0, MAX_HIST_LEN])
        self.plot_y.invertX()
        self.plot_y.setRange(xRange=[0, MAX_HIST_LEN], yRange=[0, 1000])
        self.plot_xy.setRange(xRange=[0, 1000], yRange=[0, 1000])
        self.hplot_x = self.plot_x.plot(pen='y')
        self.hplot_y = self.plot_y.plot(pen='y')
        self.hplot_xy = self.plot_xy.plot(pen='y')


        #### Set Data  #####################
        self.x = np.linspace(0, MAX_HIST_LEN + 1, num = MAX_HIST_LEN)
        self.leak_factor = 0.01

        self.counter = 0
        self.fps = 0.
        self.lastupdate = time.time()

        self.ser_available = False
        self.ser = serial.Serial()
        self.ser.port = '/dev/tty.usbmodem1423'
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS #number of bits per bytes
        self.ser.parity = serial.PARITY_NONE #set parity check: no parity
        self.ser.stopbits = serial.STOPBITS_ONE #number of stop bits
        self.ser.timeout = 1            #non-block read
        self.ser.xonxoff = False     #disable software flow control
        self.ser.rtscts = False     #disable hardware (RTS/CTS) flow control
        self.ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
        self.ser.writeTimeout = 2     #timeout for write

        try:
            self.ser.open()
            self.ser_available = True
        except serial.serialutil.SerialException:
            print("Could not open port.")

        self.sensor_x = deque(MAX_HIST_LEN * [0], MAX_HIST_LEN)
        self.sensor_y = deque(MAX_HIST_LEN * [0], MAX_HIST_LEN)

        #### Start  #####################
        if (self.ser_available): self.ser.reset_input_buffer()
        self._update()

    def _update(self):
        if (self.ser_available): self.updateSensorValue()
        if (self.ser_available): self.updateSensorValue()

        self.hplot_x.setData(self.sensor_x, self.x)
        self.hplot_y.setData(self.sensor_y)
        self.hplot_xy.setData([self.sensor_x[0]], [self.sensor_y[0]], pen = None, symbol = 'o')

        now = time.time()
        dt = (now-self.lastupdate)
        if dt <= 0:
            dt = 0.000000000001
        fps2 = 1.0 / dt
        self.lastupdate = now
        self.fps = self.fps * 0.9 + fps2 * 0.1
        tx = 'Mean Frame Rate:  {fps:0.0f} FPS'.format(fps=self.fps )
        self.label.setText(tx)
        QtCore.QTimer.singleShot(1, self._update)
        self.counter += 1

    def updateSensorValue(self):
#            print(self.ser.in_waiting)
#        if (self.ser.in_waiting > 10):
            readback = self.ser.readline()
            readback_split = readback.decode().split(',')
            try:
                if (len(readback_split) != 2):
                    return
                sensor_num = readback_split[0]
                range_val_raw = int(readback_split[1])
                if (range_val_raw > 1000):
                    range_val_raw = 1000
            except ValueError:
                print(readback)
                return
            if (sensor_num == '0'):
                filt_val = int(self.sensor_x[0] * self.leak_factor + range_val_raw * (1 - self.leak_factor))
                self.sensor_x.appendleft(filt_val)
            elif (sensor_num == '1'):
                filt_val = int(self.sensor_y[0] * self.leak_factor + range_val_raw * (1 - self.leak_factor))
                self.sensor_y.appendleft(filt_val)

    def close(self):
        self.ser.close()

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    thisapp = App()
    thisapp.show()
    sys.exit(app.exec_())
