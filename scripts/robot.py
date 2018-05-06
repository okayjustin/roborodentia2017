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
from timeit import default_timer as timer

# Thresholds for changing areas
kAREA_THRESHOLD = 950
kAREA_HYST = 50

MAX_PWM_CYCLES = 2047
BUTTON_PWM_CYCLES = 1700

# CALIBRATION VALUES, offset and scale
RANGE_S = 0.965025066
RANGE_O = -3.853474266
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
    check_btn_cmd = 'Z\n'.encode('utf-8')
    fire_left_cmd = 'LL\n'.encode('utf-8')
    fire_right_cmd = 'LR\n'.encode('utf-8')

    # Time delay lengths
    TIME_LAUNCHER = 3
    TIME_LOAD = 0.5

    # Set point positions
    POSX_LEFT_SIDE = 190
    POSX_RAMP_LEFT = 560
    POSX_RAMP_RIGHT = FIELD_XMAX - POSX_RAMP_LEFT
    POSX_RIGHT_SIDE = FIELD_XMAX - POSX_LEFT_SIDE
    POSX_START = 2150

    POSY_FRONT = 860
    POSY_CENTER = 620
    POSY_PREBACK = 350
    POSY_BACK = 280

    POS_BL_PRELOAD  = [POSX_LEFT_SIDE,  POSY_PREBACK, 0]
    POS_BL_LOAD     = [POSX_LEFT_SIDE,  POSY_BACK,    0]
    POS_FL_LOAD     = [POSX_LEFT_SIDE,  POSY_FRONT,   0]

    POS_RAMP_LEFT   = [POSX_RAMP_LEFT,  POSY_CENTER,  0]
    POS_RAMP_RIGHT  = [POSX_RAMP_RIGHT, POSY_CENTER,  0]

    POS_BR_PRELOAD  = [POSX_RIGHT_SIDE, POSY_PREBACK, 0]
    POS_BR_LOAD     = [POSX_RIGHT_SIDE, POSY_BACK,    0]
    POS_FR_LOAD     = [POSX_RIGHT_SIDE, POSY_FRONT,   0]

    POS_FIRE_A      = [POSX_RAMP_RIGHT, POSY_CENTER, -0.020]
    POS_FIRE_B      = [POSX_RAMP_RIGHT, POSY_CENTER, +0.070]
    POS_FIRE_C      = [POSX_RAMP_LEFT,  POSY_CENTER, -0.005]
    POS_FIRE_D      = [POSX_RAMP_LEFT,  POSY_CENTER, +0.005]

    epsilon_x = 10
    epsilon_xdot = 5.0
    epsilon_y = 10
    epsilon_ydot = 5.0
    epsilon_th = np.radians(1)
    epsilon_thdot = np.radians(1)

    motor_scale = [1.0, 0.9, 1.0, 0.9]

    # PID gains          x,    y,    th
    #    Kp = np.array([0.05, 0.05, 8.00])
    #    Ki = np.array([0.00, 0.00, 0.00])
    #    Kd = np.array([0.0001, 0.0001, 0.02])
    Kp = np.array([0.008, 0.008, 3.00])
    Ki = np.array([0.002, 0.002, 0.20])
    Kd = np.array([0.0007, 0.0007, 0.30])


    def getTime(self):
        if (self.sim):
            return self.time
        else:
            return timer()

    def incTime(self):
        self.time += self.dt

    # State machine
    def stateMachineCycle(self):
        self.sm_state_prev = self.sm_state

        # Check if time exceeds 3 minutes
        if False: #(time > 3 minutes):
            self.sm_state = 0
        elif ((self.button_pressed) and (self.sm_state != 0)):
            # If debug button pressed, go to reset
            self.sm_state = 0
        else:
            # Update the state to the next state
            self.sm_states[self.sm_state](self, 0)

        print("State: %d" % self.sm_state)

        # Run the next state
        self.sm_states[self.sm_state](self, 1)

    # State functions
    def state0(self, func):  # Reset state
        if (func == 0):
            if (self.button_pressed):
                self.sm_state = 11
                print("Going to state 1")
                quit()
                self.sm_time_start = self.getTime()
        elif (func == 1):
            pass

    def state1(self, func): # Fire the five balls that the robot starts with
        time_len = self.TIME_LAUNCHER # second
        if (func == 0):
            if (self.getTime() - self.sm_time_start > time_len): # Time condition
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            if (self.sm_state_prev == 0):
                self.console.ser.write(self.fire_left_cmd)

    def state2(self, func): # Move diagonally to BL preload position
        if (func == 0):
            x_err = abs(self.pid_e[0])
            x_dot = abs(self.state[1].curVal())
            if ((x_err < self.epsilon_x) and (x_dot < self.epsilon_xdot)): # e(x)=0, v(x)=0
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            self.setDesired(self.POS_BL_PRELOAD)

    def state3(self, func): # Move backwards to start loading BL
        if (func == 0):
            y_err = abs(self.pid_e[1])
            y_dot = abs(self.state[3].curVal())
            if ((y_err < self.epsilon_y) and (y_dot < self.epsilon_ydot)): # e(y)=0, v(y)=0
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            self.setDesired(self.POS_BL_LOAD)

    def state4(self, func): # Wait to finish loading BL
        time_len = self.TIME_LOAD # second
        if (func == 0):
            if (self.getTime() - self.sm_time_start > time_len): # Time condition
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            pass

    def state5(self, func): # Move forward to load FL
        if (func == 0):
            y_err = abs(self.pid_e[1])
            print(y_err)
            print(self.state[2].curVal())
            print(self.state[6].curVal())
            y_dot = abs(self.state[3].curVal())
            if ((y_err < self.epsilon_y) and (y_dot < self.epsilon_ydot)): # e(y)=0, v(y)=0
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            self.setDesired(self.POS_FL_LOAD)

    def state6(self, func): # Wait to finish loading FL
        time_len = self.TIME_LOAD # second
        if (func == 0):
            if (self.getTime() - self.sm_time_start > time_len): # Time condition
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            pass

    def state7(self, func): # Move diagonally to left of left ramp
        if (func == 0):
            x_err = abs(self.pid_e[0])
            x_dot = abs(self.state[1].curVal())
            y_err = abs(self.pid_e[1])
            y_dot = abs(self.state[3].curVal())
            if ((x_err < self.epsilon_x) and (x_dot < self.epsilon_xdot) and # e(x)=0, v(x)=0
                (y_err < self.epsilon_y) and (y_dot < self.epsilon_ydot)): # e(y)=0, v(y)=0
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            self.setDesired(self.POS_RAMP_LEFT)

    def state8(self, func): # Rotate to C/D target + fire balls
        time_len = self.TIME_LAUNCHER # second
        if (func == 0):
            if (self.getTime() - self.sm_time_start > time_len): # Time condition
                if (self.target_cycle == 0):
                    self.target_cycle = 1
                else:
                    self.target_cycle = 0
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            if (self.target_cycle == 0):
                self.setDesired(self.POS_FIRE_D)
            else:
                self.setDesired(self.POS_FIRE_C)

    def state9(self, func): # Rotate to 0 degrees
        if (func == 0):
            th_err = abs(self.pid_e[2])
            th_dot = abs(self.state[4].curVal())
            if ((th_err < self.epsilon_th) and (th_dot < self.epsilon_thdot)): # e(th)=0, v(th)=0
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            self.setDesired(self.POS_RAMP_LEFT)

    def state10(self, func): # Go to right of right ramp
        if (func == 0):
            x_err = abs(self.pid_e[0])
            if (x_err < self.epsilon_x): # e(x)=0
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            self.setDesired(self.POS_RAMP_RIGHT)

    def state11(self, func): # Move diagonally to BR preload position
        if (func == 0):
            x_err = abs(self.pid_e[0])
            x_dot = abs(self.state[1].curVal())
            if ((x_err < self.epsilon_x) and (x_dot < self.epsilon_xdot)): # e(x)=0, v(x)=0
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            self.setDesired(self.POS_BR_PRELOAD)

    def state12(self, func): # Move backwards to start loading BR
        if (func == 0):
            y_err = abs(self.pid_e[1])
            y_dot = abs(self.state[3].curVal())
            if ((y_err < self.epsilon_y) and (y_dot < self.epsilon_ydot)): # e(y)=0, v(y)=0
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            self.setDesired(self.POS_BR_LOAD)

    def state13(self, func): # Wait to finished loading BR
        time_len = self.TIME_LOAD # second
        if (func == 0):
            if (self.getTime() - self.sm_time_start > time_len): # Time condition
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            pass

    def state14(self, func): # Move forward to load FR
        if (func == 0):
            y_err = abs(self.pid_e[1])
            y_dot = abs(self.state[3].curVal())
            if ((y_err < self.epsilon_y) and (y_dot < self.epsilon_ydot)): # e(y)=0, v(y)=0
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            self.setDesired(self.POS_FR_LOAD)

    def state15(self, func): # Wait to finish loading FR
        time_len = self.TIME_LOAD # second
        if (func == 0):
            if (self.getTime() - self.sm_time_start > time_len): # Time condition
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            pass

    def state16(self, func): # Move diagonally to right of right ramp
        if (func == 0):
            x_err = abs(self.pid_e[0])
            x_dot = abs(self.state[1].curVal())
            y_err = abs(self.pid_e[1])
            y_dot = abs(self.state[3].curVal())
            if ((x_err < self.epsilon_x) and (x_dot < self.epsilon_xdot) and  # e(x)=0, v(x)=0
                (y_err < self.epsilon_y) and (y_dot < self.epsilon_ydot)): # e(y)=0, v(y)=0
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            self.setDesired(self.POS_RAMP_RIGHT)

    def state17(self, func): # Rotate to A/B target and fire
        time_len = self.TIME_LAUNCHER # second
        if (func == 0):
            if (self.getTime() - self.sm_time_start > time_len): # Time condition
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            if (self.target_cycle == 0):
                self.setDesired(self.POS_FIRE_B)
            else:
                self.setDesired(self.POS_FIRE_A)

    def state18(self, func): # Rotate to 0 degrees
        if (func == 0):
            th_err = abs(self.pid_e[2])
            th_dot = abs(self.state[4].curVal())
            if ((th_err < self.epsilon_th) and (th_dot < self.epsilon_thdot)): # e(th)=0, v(th)=0
                self.sm_state += 1
                self.sm_time_start = self.getTime()
        elif (func == 1):
            self.setDesired(self.POS_RAMP_RIGHT)

    def state19(self, func): # Go to left of left ramp
        if (func == 0):
            x_err = abs(self.pid_e[0])
            if (x_err < self.epsilon_x): # e(x)=0
                self.sm_state = 2   # Go back to state 2
                self.sm_time_start = self.getTime()
        elif (func == 1):
            self.setDesired(self.POS_RAMP_LEFT)

    # State mapping
    sm_states = { 0 : state0,
                  1 : state1,
                  2 : state2,
                  3 : state3,
                  4 : state4,
                  5 : state5,
                  6 : state6,
                  7 : state7,
                  8 : state8,
                  9 : state9,
                 10 : state10,
                 11 : state11,
                 12 : state12,
                 13 : state13,
                 14 : state14,
                 15 : state15,
                 16 : state16,
                 17 : state17,
                 18 : state18,
                 19 : state19
    }

    def __init__(self, simulate, field_area_init):
        self.sim = simulate
        if (self.sim):
            import robotsim
            state_start = [self.POSX_START, 0, self.POSY_CENTER, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.env = robotsim.SimRobot(train = 'sim', state_start = state_start)

        self.motorSpeeds = np.array([0, 0, 0, 0], dtype='f')     # speeds range from -1 to 1

        # robot setup
        self.max_hist_len = 100
        self.dt = 0.05
        self.time = 0

        self.full_th = 0
        self.th_part = 0
        self.th_start = 0

        ''' States
        [0]: x
        [1]: x dot
        [2]: y
        [3]: y dot
        [4]: th
        [5]: th dot
        [6]: desired x
        [7]: desired y
        [8]: desired theta
        [9]: Hopper BL state
        [10]: Hopper BR state
        [11]: Hopper FR state:
        [12]: field area (-1 for left, 0 for center platform, 1 for right)
        '''
        self.state = [RunningStat(3) for x in range(0,13)]
        self.state[12].push(field_area_init)

        ''' Control array U. All values range from -2 to 2
        [0]: x translation
        [1]: y translation
        [2]: angle control
        [3]: launch command (-1 for fire left, +1 for fire right, 0 for nothing)
        '''
        self.u = np.zeros(5)

        # Initialize PID vars
        self.pid_e = np.zeros(3)    # Error
        self.pid_int = np.zeros(3)  # Integral

        # State machine vars
        if (self.sim):
            self.sm_state = 11
        else:
            self.sm_state = 0
        self.sm_state_prev = 0
        self.button_pressed = False
        self.target_cycle = 0 # 0 for targets B/D, 1 for targets A/C
        self.sm_time_start = self.getTime()

        # Sensor array. Each contains a running history, offset factor, scaling factor
        # Rangefinders: Units of mm
        # Gyro: Units of degrees per second
        self.sensors = [[RunningStat(self.max_hist_len), RANGE_S, RANGE_O], # RF 0     0
                        [RunningStat(self.max_hist_len), RANGE_S, RANGE_O], # RF 1     1
                        [RunningStat(self.max_hist_len), RANGE_S, RANGE_O], # RF 2     2
                        [RunningStat(self.max_hist_len), RANGE_S, RANGE_O], # RF 3     3
                        [RunningStat(self.max_hist_len),1/10000.,     0.0]] # Heading
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
        if (self.sim):
            return 0
        else:
            return self.console.openSerial()

    def close(self):
        if (not self.sim):
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

    # Calculates a control array (u) based on the current state using PID algorithm
    def calcU(self):
        # Get state
        x      = self.state[0].curVal()
        y      = self.state[2].curVal()
        th     = self.state[4].curVal()
        x_des  = self.state[6].curVal()
        y_des  = self.state[7].curVal()
        th_des = self.state[8].curVal()

        setpoint = np.array([x_des, y_des, th_des])
        measval = np.array([x, y, th])

#        print("PID set point: ", end='')
#        print(setpoint)
#        print("PID meas point: ", end='')
#        print(measval)

        new_error = setpoint - measval
        self.pid_int = self.pid_int + new_error*self.dt
        der = (new_error - self.pid_e) / self.dt
        self.u = np.multiply(self.Kp, new_error) + np.multiply(self.Ki, self.pid_int) + np.multiply(self.Kd, der)
        self.pid_e = new_error

        # Disable x and y movement when rotated
        if (th_des != 0):
            self.u[0] = 0
            self.u[1] = 0
        self.u[0] = 0
        self.u[2] = 0
        print("PID Error: ", end='')
        print(self.pid_e)
        print("PID Ctrls: ", end='')
        print(self.u)

    def execute(self):
        if (self.sim):
            self.env.step([self.u[0], -self.u[1], -self.u[2]])
            #self.env.step([-1, 0, -self.u[2]])
        else:
            self.mechanumCommand(self.u[0], self.u[1], self.u[2])

    def render(self):
        self.env.render()

    def updateSensorValue(self):
        if (self.sim):
            self.env.updateObservation()
            sim_obs = self.env.obs

            # Simulate rangefinder readings
            for i in range(0, 4):
                self.sensors[i][0].push(sim_obs[i*2])

            # Simulate heading, no need to simulated individual magnetometer and accel readings
            sim_th = np.arctan2(sim_obs[9], sim_obs[8])

        else:
            try:
                # Check if button has been pressed
                self.console.ser.reset_input_buffer()
                self.console.ser.write(self.check_btn_cmd)
                data = self.console.ser.readline()
                if data == b'0\n':
                    self.button_pressed = True
                else:
                    self.button_pressed = False

                # Request data
                while True:
                    self.console.ser.reset_input_buffer()
                    self.console.ser.write(self.data_cmd)
                    data = self.console.ser.read(self.num_sensors * 2 + 1)
                    if (len(data) == self.num_sensors * 2 + 1):
                        if (data[-1] == 10):
                            break
                    else:
                        print("Malformed UART data. Len: %d. Retrying..." % len(data))
                        pass
            except OSError:
                print("Error")

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
        y_front = self.sensors[0][0].curVal()
        x_left  = self.sensors[1][0].curVal()
        y_back  = self.sensors[2][0].curVal()
        x_right = self.sensors[3][0].curVal()
        if (not self.sim):
            heading = self.sensors[4][0].curVal()
            print(np.degrees(heading))

        x       = self.state[0].curVal()
        xdot    = self.state[1].curVal()
        y       = self.state[2].curVal()
        ydot    = self.state[3].curVal()
        th      = self.state[4].curVal()
        thdot   = self.state[5].curVal()
        x_des   = self.state[6].curVal()
        y_des   = self.state[7].curVal()
        th_des  = self.state[8].curVal()
        hopbl   = self.state[9].curVal()
        hopbr   = self.state[10].curVal()
        hopfr   = self.state[11].curVal()
        field_area = self.state[12].curVal()

        #-------------------------------------------------------------
        # Update field area
        if (field_area == -1): # Left field
            if (x_left > kAREA_THRESHOLD):
                new_field_area = 0
            else:
                new_field_area = -1
        elif (field_area == 1): # Right field
            if (x_right > kAREA_THRESHOLD):
                new_field_area = 0
            else:
                new_field_area = 1
        else:                   # Center field
            if (x_left < kAREA_THRESHOLD):
                new_field_area = -1
            elif (x_right < kAREA_THRESHOLD):
                new_field_area = 1
            else:
                new_field_area = 0

        #-------------------------------------------------------------
        # Update x, and xdot states
        if (new_field_area == -1):      # Left
            new_x = x_left + LEN_X / 2
        elif (new_field_area == 1):    # Right
            new_x = FIELD_XMAX - x_right - LEN_X / 2
        else:                           # Center
            new_x = (x_left + FIELD_XMAX - x_right) / 2.0
        new_xdot = self.getVel(0)

#        print("Field area: %d" % field_area)
#        print("newX: %0.3f, actual: %0.3f error: %0.3f x_left: %0.3f, x_right: %0.3f" % (new_x, self.env.state[0], new_x - self.env.state[0], x_left, x_right))
        #-------------------------------------------------------------
        # Update y and ydot states
        new_y = (y_back + FIELD_YMAX - y_front) / 2.0
        new_ydot = self.getVel(2)

        #-------------------------------------------------------------
        # Update theta and thetadot states
        self.prev_th_part = self.th_part
        if (self.sim):
            self.th_part = sim_th
        else:
            self.th_part = heading

        # If previous theta is near the upper limit (between 90 and 180 degrees) and
        # current theta is past the 180 degree limit (which means between -90 and -180 degrees)
        if (self.prev_th_part > np.pi/2 and self.th_part < -np.pi/2):
            self.full_th += 1  # Increment full rotation count
        # If previous th is near the lower limit (between -90 and -180 degrees)
        elif (self.prev_th_part < -np.pi/2 and self.th_part > np.pi/2):
            self.full_th -= 1  # Increment full rotation count

        new_th = self.full_th*2*np.pi + self.th_part - self.th_start
        new_thdot = self.getVel(4)

        #-------------------------------------------------------------
        # Update hopper states
        new_hopbl = 0
        new_hopbr = 0
        new_hopfr = 0

        #-------------------------------------------------------------
        # Push new states
        self.state[0].push(new_x)
        self.state[1].push(new_xdot)
        self.state[2].push(new_y)
        self.state[3].push(new_ydot)
        self.state[4].push(new_th)
        self.state[5].push(new_thdot)
        self.state[9].push(new_hopbl)
        self.state[10].push(new_hopbr)
        self.state[11].push(new_hopfr)
        self.state[12].push(new_field_area)

    # Sets a new desired X,Y,Th
    def setDesired(self, coordinates):
        self.state[6].push(coordinates[0]) # X
        self.state[7].push(coordinates[1]) # Y
        self.state[8].push(coordinates[2]) # th

    # Calculates the average of forwards and backwards derivatives
    def getVel(self, stateIdx):
        deriv_len = 1
        future = self.state[stateIdx].vals[0]
        past = self.state[stateIdx].vals[deriv_len]
        return (future - past)/(deriv_len * self.dt)


    # Recalculates the starting theta position to "zero out" theta
    def initXYT(self):
        # Reset DC offset
        self.th_start = 0

        # Get the theta averaged over a number of samples
        num_pts = 100
        for i in range(0, num_pts):
            self.updateSensorValue()
            time.sleep(0.01)
        self.th_start = self.state[4].winMean()

        self.state[6].push(self.state[0].winMean())
        self.state[7].push(self.state[2].winMean())

    def printSensorVals(self):
        print("Rangefinder values:")
        for i in range(0,4):
            print("\tRF%d: %+0.3f | Var: %+0.3f" % (i, self.sensors[i][0].curVal(), self.sensors[i][0].winStdDev()))
        print("State values:")
        print("\tX: %+0.3f | Xdot: %+0.3f" % (self.state[0].curVal(), self.state[1].curVal()))
        print("\tY: %+0.3f | Ydot: %+0.3f" % (self.state[2].curVal(), self.state[3].curVal()))
        print("\tTh: %+0.3f | Thdot: %+0.3f" % (self.state[4].curVal(), self.state[5].curVal()))
        print("\tXdes: %+0.3f | Ydes: %+0.3f | Thdes: %+0.3f" % (self.state[6].curVal(), self.state[7].curVal(), self.state[8].curVal()))

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
        print("Motor speeds: ", end='')
        print(self.motorSpeeds)

        # Send motor control commands
        motorCmd = []
        for i in range(0,4):
            motorCmd.append(int(self.motorSpeeds[i] * MAX_PWM_CYCLES * self.motor_scale[i]))
        self.cmdMotor(motorCmd)

    # speed : 0 to 2047, duty cycle value out of 2047
    def cmdMotor(self, motorCmd):

        print("Motor cmd: ", end='')
        print(motorCmd)
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

