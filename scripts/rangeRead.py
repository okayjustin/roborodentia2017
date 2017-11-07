#!/usr/local/bin/python3

from robot import robot
import sys, time, statistics, threading
import numpy as np
import PyQt5.QtGui as QtGui
import PyQt5.QtCore as QtCore
import pyqtgraph as pg

AXIS_PLOT_SIZE = 400
MEDIAN_LENGTH = 30

class App(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(App, self).__init__(parent)
        pg.setConfigOptions(useOpenGL=1)  # SPEEEEEED

        # Create a robot
        self.robot = robot()

        #### Create Gui Elements ###########
        self.mainbox = QtGui.QWidget()
        self.setCentralWidget(self.mainbox)
        self.mainbox.setLayout(QtGui.QBoxLayout(QtGui.QBoxLayout.TopToBottom))
        self.hlayout = QtGui.QHBoxLayout()
        self.vlayout = QtGui.QVBoxLayout()

        # Define widgets
        self.pgcanvas = pg.GraphicsLayoutWidget()
        self.buttonLogStart = QtGui.QPushButton('Start data log')
        self.buttonLogStart.clicked.connect(self.robot.startLog)
        self.buttonLogStop = QtGui.QPushButton('Stop data log')
        self.buttonLogStop.clicked.connect(self.robot.stopLog)
        self.serialStatuslabel = QtGui.QLabel()
        self.serialStatuslabel.setText('Serial: Not connected.')
        self.serialConsole = QtGui.QLineEdit()
        self.serialConsole.returnPressed.connect(self.writeSerialConsole)
        self.buttonForward = QtGui.QPushButton('Forward')
        self.buttonForward.clicked.connect(self.robot.botCmdForwardButton)
        self.buttonStop = QtGui.QPushButton('Stop')
        self.buttonStop.clicked.connect(self.robot.botCmdStopButton)
        self.buttonReverse = QtGui.QPushButton('Reverse')
        self.buttonReverse.clicked.connect(self.robot.botCmdReverseButton)
        self.positionlabel = QtGui.QLabel()
        newfont = QtGui.QFont("courier")
        self.positionlabel.setFont(newfont)
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
        self.plot_y.setYRange(0, self.robot.max_hist_len, padding=0)
        self.plot_y_raw = self.plot_y.plot(pen='y')
        self.plot_y_hist = self.plot_y.plot( stepMode=True, fillLevel=0, brush=(0,0,255,150))

        # X range plot
        self.plot_x = self.pgcanvas.addPlot(1,1,labels={'left':'Latest sample #','bottom':'X distance(mm)'})
        self.plot_x.showGrid(1,1,255)
        self.plot_x.invertY()
        self.plot_x.setYRange(0, self.robot.max_hist_len, padding=0)
        self.plot_x_raw = self.plot_x.plot(pen='y')
        self.plot_x_kalman = self.plot_x.plot(pen='r')
        self.plot_x_hist = self.plot_x.plot( stepMode=True, fillLevel=0, brush=(0,0,255,150))

        # Position arrow
        self.abs_position_arrow = pg.ArrowItem(angle=0, tipAngle=45, headLen=15, tailLen=15, tailWidth=3, brush='y')
        self.abs_position_arrow.rotate(90)
        self.plot_xy.addItem(self.abs_position_arrow)

        # Control/data signals
        self.sensor_x_median = 0
        self.sensor_x_var = 0
        self.sensor_x_kal_var = 0
        self.sensor_y_median = 0
        self.sensor_y_var = 0
        self.x = np.linspace(0, self.robot.max_hist_len + 1, num = self.robot.max_hist_len)
        self.histbins = bins=np.linspace(0, 1000, 1000)
        self.fps = 0.
        self.lastupdate = time.time()
        self.arrow_angle = 0
        self.exiting = False
        self.sensor_update_thread = threading.Thread(target=self.updateSensorValueWorker)
        self.thread_lock = threading.Lock()

        #### Start  #####################
        self._update()

    def _update(self):
        # If no serial available, try to open a new one
        if (self.robot.ser_available == False):
            self.serialStatuslabel.setText('Serial: Not connected.')
            if (self.sensor_update_thread.is_alive()):
                pass
            self.robot.openSerial()
            if (self.robot.ser_available):
                self.serialStatuslabel.setText('Serial: Connected.')
                self.sensor_update_thread = threading.Thread(target=self.updateSensorValueWorker)
                self.sensor_update_thread.start()

#            self.robot.angleControlTest(90.0)

        # Acquire thread lock to get data from thread
        self.thread_lock.acquire()
        sensor_x_deque = self.robot.sensor_x
        sensor_x_kalman_deque = self.robot.sensor_x_kalman
        sensor_y_deque = self.robot.sensor_y
        dt_deque = self.robot.dt
        sensor_mag_homed = self.robot.sensor_mag_homed
        sensor_mag_ref = self.robot.sensor_mag_ref
        sensor_mag = self.robot.sensor_mag
        sensor_accel_x_deque = self.robot.sensor_accel_x
        sensor_accel_y_deque = self.robot.sensor_accel_y
        sensor_accel_z_deque = self.robot.sensor_accel_z
        self.thread_lock.release()

        # Convert deques to lists for easier processing
        sensor_x_list = list(sensor_x_deque)
        sensor_x_kalman_list = list(sensor_x_kalman_deque)
        sensor_y_list = list(sensor_y_deque)
        dt_list = list(dt_deque)

        # Calculate some stuff
        plot_x_y,plot_x_x = self.reducedHistogram(sensor_x_list, self.histbins)
        plot_y_y,plot_y_x = self.reducedHistogram(sensor_y_list, self.histbins)
        self.sensor_x_median = statistics.median(sensor_x_kalman_list[:MEDIAN_LENGTH])
        self.sensor_x_var = np.var(sensor_x_list)
        self.sensor_x_kal_var = np.var(sensor_x_kalman_list)
        self.sensor_y_median = statistics.median(sensor_y_list[:MEDIAN_LENGTH])
        self.sensor_y_var = np.var(sensor_y_list)

        # Update plots
        self.plot_x_raw.setData(sensor_x_list, self.x)
        self.plot_x_kalman.setData(sensor_x_kalman_list, self.x)
        self.plot_y_raw.setData(sensor_y_list, self.x)
        self.plot_x_hist.setData(plot_x_x,plot_x_y)
        self.plot_y_hist.setData(plot_y_x,plot_y_y)
        self.abs_position_arrow.setPos(self.sensor_x_median,self.sensor_y_median)
        self.setArrowAngle(90.0 - sensor_mag_homed)


        # Update the labels on the side
        x_pos_str =         'Median X: \t%d \tmm\n' % self.sensor_x_median
        x_var_str =         'Var X: \t\t%0.2f \tmm^2\n' % self.sensor_x_var
        x_kal_var_str =     'Var Kal X: \t%0.2f \tmm^2\n' % self.sensor_x_kal_var
        x_var_ratio_str =   'Var ratio X: \t%0.2f\n' % (self.sensor_x_kal_var / (self.sensor_x_var + 0.00000001))
        y_pos_str =         '\nMedian Y: \t%d \tmm\n' % self.sensor_y_median
        y_var_str =         'Var y: \t\t%0.2f \tmm^2\n' % self.sensor_y_var
        angle_str =         '\nAngle: \t\t%0.1f \tdeg\n' % (sensor_mag_homed)
        raw_angle_str =     'Raw angle: \t%0.1f \tdeg\n' % (sensor_mag[0])
        ref_angle_str =     'Ref angle: \t%0.1f \tdeg\n' % (sensor_mag_ref)
        x_accel_str =       '\nX accel: \t%+0.5f g\n' % sensor_accel_x_deque[0]
        y_accel_str =       'Y accel: \t%+0.5f g\n' % sensor_accel_y_deque[0]
        z_accel_str =       'Z accel: \t%+0.5f g\n' % sensor_accel_z_deque[0]
        data_rate_str =     '\nData rate: \t%0.1f \tHz\n' % (1000.0 / statistics.mean(dt_list))
        data_rate_per_str = 'Data rate per: \t%0.3f \tms\n' % (statistics.mean(dt_list))
        data_rate_var_str = 'Data rate var: \t%0.4f \tms\n' % (statistics.variance(dt_list))


        positionlabel_str = x_pos_str + x_var_str + x_kal_var_str + x_var_ratio_str \
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
        if (not self.exiting):
            QtCore.QTimer.singleShot(1, self._update)


    def updateSensorValueWorker(self):
        if (not self.exiting):
            print("Sensor update worker started.")
        while (not self.exiting):
            self.thread_lock.acquire()
            self.robot.updateSensorValue()
            self.thread_lock.release()

    # Gets the histogram but without empty bins
    def reducedHistogram(self, data_list, bins):
        y,x = np.histogram(data_list, bins)
        nonzero_indices = np.nonzero(y)
        y_reduced = y[nonzero_indices[0][0]:nonzero_indices[0][-1] + 1]
        x_reduced = x[nonzero_indices[0][0]:nonzero_indices[0][-1] + 2]
        return y_reduced, x_reduced

    def setArrowAngle(self, angle):
        self.abs_position_arrow.rotate(angle - self.arrow_angle)
        self.arrow_angle = angle

    def writeSerialConsole(self):
        cmd_str = self.serialConsole.text()
        self.serialConsole.setText('')
        if (self.robot.ser_available):
            cmd_str += '\r'
            self.robot.ser.write(cmd_str.encode('utf-8'))

    def closeEvent(self, *args, **kwargs):
        self.exiting = True
        super(QtGui.QMainWindow, self).closeEvent(*args, **kwargs)
        if (self.robot.ser_available): self.robot.closeSerial()
        if (self.robot.data_log_enable): self.stopLog()



if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    thisapp = App()
    thisapp.show()
    sys.exit(app.exec_())
