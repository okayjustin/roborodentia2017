#!/usr/local/bin/python3

from helper_funcs import *
import numpy as np
from console import *
import struct
import time
from timeit import default_timer as timer

class Robot():
    '''
    desc
    '''

    # Maximum allowed motor speed value for UART command
    MAX_PWM_CYCLES = 2047

    # CALIBRATION VALUES, offset and scale
    RANGE_S = 0.965025066
    RANGE_O = -3.853474266
    MAG_S_X = 1 / 545.1754644628942
    MAG_S_Y = 1 / 550.0729613545005
    MAG_S_Z = 1 / 421.44904386765774
    MAG_O_X = -32 * -MAG_S_X
    MAG_O_Y = 432 * -MAG_S_Y
    MAG_O_Z = 79 * -MAG_S_Z
    ACC_S_X = 1 / 1030.9868342172633
    ACC_S_Y = 1 / 1059.8985318881996
    ACC_S_Z = 1 / 1063.2448503229036
    ACC_O_X = 30 * -ACC_S_X
    ACC_O_Y = 0 * -ACC_S_Y
    ACC_O_Z = -76 * -ACC_S_Z

    # Corrects setpoint offset error in networks
    X_OFFSET = 10
    Y_OFFSET = 10

    # Robot dimensions
    LEN_X = 315.0      # length of the robot left to right
    LEN_Y = 275.0      # length of the robot front to back
    FIELD_XMAX = 2438.4 # Maximum x dimension in mm
    FIELD_YMAX = 1219.2  # Maximum y dimension in mm
    WHEEL_R =  30.0   # Wheel radius in mm
    FRONT_BACK_WHEEL_DIST = 238.7  # Distance between the centers of front and back wheels in mm
    LEFT_RIGHT_WHEEL_DIST = 251.4  # Distance between the centers of left and right wheels in mm

    # Motor constants
    wheel_max_thetadotdot = 430 * 0.2  # Max wheel rotational acceleration in rad/s^2
    wheel_max_thetadot = 24.46 * 0.5   # Max wheel rotational velocity in rad/s
    friction_constant = wheel_max_thetadotdot / wheel_max_thetadot # Friction constant in 1/s

    # UART Commands
    sense_cmd = 'A\n'.encode('utf-8')           # Queue rangefinder read
    data_cmd = 'B\n'.encode('utf-8')            # Request sensor data packet
    check_btn_cmd = 'Z\n'.encode('utf-8')       # Request debug button latch state
    fire_left_cmd = 'LL\n'.encode('utf-8')      # Fire balls from left hopper
    fire_right_cmd = 'LR\n'.encode('utf-8')     # Fire balls from right hopper

    # Time step per cycle
    dt = 0.05

    # Returns either real or simulated time
    def getTime(self):
        if (self.sim):
            return self.time
        else:
            return timer()

    # Increment the simulated time
    def incTime(self):
        self.time += self.dt

    def __init__(self, simulate):
        # If running simulation
        self.sim = simulate
        if (self.sim):
            import robotsim
            state_start = [self.POSX_START, 0, self.POSY_CENTER, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.env = robotsim.SimRobot(train = 'sim', state_start = state_start)

        # Robot setup
        self.max_hist_len = 100
        self.time = 0

        # For tracking the robot angle
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
        [8]: desired theta '''
        self.num_states = 9
        self.state = [RunningStat(5) for x in range(0, self.num_states)]

        ''' Control array U. All values range from -2 to 2
        [0]: x translation
        [1]: y translation
        [2]: angle control
        [3]: launch command (-1 for fire left, +1 for fire right, 0 for nothing) '''
        self.u = np.zeros(5)

        # Desired motor speeds as a fraction of maximum speed. [FL, BL, BR, FR]
        self.motorSpeeds = np.array([0, 0, 0, 0], dtype='f')     # speeds range from -1 to 1

        ''' Sensor array. Each contains a running history, offset factor, scaling factor
        Rangefinders: Units of mm
        Magnetometer: Units of  uTesla?. Arbitrary since converted to heading later
        Accelerometer: Units of g's (gravity)
        Gyro: Units of degrees per second '''
        self.sensors = [[RunningStat(self.max_hist_len), self.RANGE_S, self.RANGE_O], # RF 0     0
                        [RunningStat(self.max_hist_len), self.RANGE_S, self.RANGE_O], # RF 1     1
                        [RunningStat(self.max_hist_len), self.RANGE_S, self.RANGE_O], # RF 2     2
                        [RunningStat(self.max_hist_len), self.RANGE_S, self.RANGE_O], # RF 3     3
                        [RunningStat(self.max_hist_len), self.MAG_S_X, self.MAG_O_X], # MAG X    4
                        [RunningStat(self.max_hist_len), self.MAG_S_Y, self.MAG_O_Y], # MAG Y    5
                        [RunningStat(self.max_hist_len), self.MAG_S_Z, self.MAG_O_Z], # MAG Z    6
                        [RunningStat(self.max_hist_len), self.ACC_S_X, self.ACC_O_X], # ACCEL X  7
                        [RunningStat(self.max_hist_len), self.ACC_S_Y, self.ACC_O_Y], # ACCEL Y  8
                        [RunningStat(self.max_hist_len), self.ACC_S_Z, self.ACC_O_Z]] # ACCEL Z  9
        self.num_sensors = len(self.sensors)

        # Track debug button state
        self.button_pressed = False

        # Class to handle serial link operations
        self.console = SerialConsole()

        # Log data to file
        self.data_log_enable = False

    # Opens serial session with robot microcontroller
    def openSerial(self):
        if (self.sim):
            return 0
        else:
            return self.console.openSerial()

    # Close serial session and tensorflow sessions
    def close(self):
        if (not self.sim):
            self.console.close()
        try:
            self.angle_ann.close()
            self.transx_ann.close()
            self.transy_ann.close()
        except:
            pass

    # Start tensorflow sessions and load network models
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

    # Forward step through ANNs to produce u array
    def predict(self, obs_sets):
        # Predict angle u
        u_angle = self.angle_ann.predict(obs_sets[0])
#        print("Th: %+0.3f | Thdot: %+0.3f | %+0.3f" % \
#                (np.degrees(self.state[4].curVal()), np.degrees(self.state[5].curVal()), u_angle))

        # Predict x translation u
        u_transx = self.transx_ann.predict(obs_sets[1])

        # Predict y translation u
        u_transy = self.transy_ann.predict(obs_sets[2])

        # Concatenate into u array and clip values
        u = np.array([u_angle, u_transx, u_transy])
        u = np.clip(u, -2., 2.)
        return u

    # Execute u on robot
    def execute(self, u):
        if (self.sim):
            self.env.step([u[0], -u[1], -u[2]])
        else:
            self.mechanumCommand(u[0], u[1], u[2])

    # Get sensor data, parse, apply calibration, update state array
    def getSensorVals(self):
        try:
            # Check if button has been pressed
            self.console.ser.reset_input_buffer()
            self.console.ser.write(self.check_btn_cmd)
            data = self.console.ser.readline()
            if data == b'0\n':
                self.button_pressed = True
            else:
                self.button_pressed = False

            # Request sensor data packet, keep trying until success
            while True:
                self.console.ser.reset_input_buffer()
                self.console.ser.write(self.data_cmd)
                data = self.console.ser.read(self.num_sensors * 2 + 1)

                # Check for correct data length
                if (len(data) == self.num_sensors * 2 + 1):
                    # If last character received is newline, break
                    if (data[-1] == 10):
                        break
                else:
#                        print("Malformed UART data. Len: %d. Retrying..." % len(data))
                    pass
        except OSError:
            print("OSError")

        # Queue next rangefinder measurement
        self.console.ser.write(self.sense_cmd)

        # Parse sensor data and apply calibration values
        sensor_vals = np.array([])
        for i in range(0, self.num_sensors):
            val = int.from_bytes(data[i*2:i*2+2], byteorder='big', signed=True)
            # Calibrated value = raw data * scaling factor + offset
            sensor_vals.append(val * self.sensors[i][1] + self.sensors[i][2])

#        # Log data
#        if (self.data_log_enable):
#            self.data_log += '%0.1f, %0.1f, %d, %d, \n' \
#                    % (dt, mag_val_raw, rangeX_val_raw, rangeY_val_raw)

        return sensor_vals

    def updateSensors(self, sensor_vals):
        for i in range(0, self.num_sensors):
            val = sensor_vals[i]
            # Calibrated value = raw data * scaling factor + offset
            self.sensors[i][0].push(val)

    # Update state array
    def updateState(self):
        # Rangefinder distances
        range_front = self.sensors[0][0].curVal()
        range_left  = self.sensors[1][0].curVal()
        range_rear  = self.sensors[2][0].curVal()
        range_right = self.sensors[3][0].curVal()

        # IMU measurements
        if (not self.sim):
            magx    = self.sensors[4][0].curVal()
            magy    = self.sensors[5][0].curVal()
            magz    = self.sensors[6][0].curVal()
            accelx  = self.sensors[7][0].curVal()
            accely  = self.sensors[8][0].curVal()
            accelz  = self.sensors[9][0].curVal()

        # Assign new x and y states
        new_x = range_left #+ self.LEN_X / 2
        new_y = range_front #+ self.LEN_Y / 2

        #-------------------------------------------------------------
        # Update theta and thetadot states
        self.prev_th_part = self.th_part
        if (self.sim):
            self.th_part = sim_th
        else:
            self.th_part = self.calcTiltCompass(magx, magy, magz, accelx, accely, accelz)

        # If previous theta is near the upper limit (between 90 and 180 degrees) and
        # current theta is past the 180 degree limit (which means between -90 and -180 degrees)
        if (self.prev_th_part > np.pi/2 and self.th_part < -np.pi/2):
            self.full_th += 1  # Increment full rotation count

        # If previous th is near the lower limit (between -90 and -180 degrees)
        elif (self.prev_th_part < -np.pi/2 and self.th_part > np.pi/2):
            self.full_th -= 1  # Increment full rotation count

        # Calculate the new theta
        new_th = self.full_th*2*np.pi + self.th_part - self.th_start

        #-------------------------------------------------------------
        # Push new x, y, theta states
        self.state[0].push(new_x)
        self.state[2].push(new_y)
        self.state[4].push(new_th)

        # Update x, y, theta velocities
        new_xdot = self.getVel(0)
        new_ydot = self.getVel(2)
        new_thdot = self.getVel(4)
        self.state[1].push(new_xdot)
        self.state[3].push(new_ydot)
        self.state[5].push(new_thdot)

    # Create observations for the various networks
    def getObservation(self):
        x = self.state[0].curVal()
        xdot = self.state[1].winMean()
        y = self.state[2].curVal()
        ydot = self.state[3].winMean()
        th = self.state[4].curVal()
        thdot = self.state[5].winMean()
        xdes = self.state[6].curVal()
        ydes = self.state[7].curVal()
        thdes = self.state[8].curVal()

        # The observations are the deltas between the current and desired states
        obs_angle = np.array([np.cos(th - thdes), np.sin(th - thdes), thdot])
        obs_transx = np.array([x - xdes, xdot])
        obs_transy = np.array([y - ydes, ydot])
        obs_all = np.concatenate((obs_transx, obs_transy, obs_angle))

        obs_sets = np.array([obs_angle, obs_transx, obs_transy, obs_all])
        return obs_sets

# --------------------------------------------- Support ---------------

    # Stop all robot drive motors
    def halt(self):
        self.mechanumCommand(0, 0, 0)

    # Render robot
    def render(self):
        self.env.render()

    def calcTiltCompass(self, magx, magy, magz, accelx, accely, accelz):
        # Returns a heading from +pi to -pi
        # Tilt compensated heading calculation
        # pitch = np.arcsin(-accelx)
        # if (np.cos(pitch) == 0.):
        #     pitch = 0.
        # roll = np.arcsin(accely / np.cos(pitch))
        # # print("Pitch: %f Roll: %f" % (np.degrees(pitch), np.degrees(roll)))
        # xh = magx * np.cos(pitch) + magz * np.sin(pitch)
        # yh = magx * np.sin(roll) * np.sin(pitch) + magy * np.cos(roll) - \
        #         magz * np.sin(roll) * np.cos(pitch)
        # th = np.arctan2(yh, xh)
        th = np.arctan2(magy, magx)
        return th

    # Sets a new desired X,Y,Th
    def setDesired(self, coordinates):
        self.state[6].push(coordinates[0]) # X
        self.state[7].push(coordinates[1]) # Y
        self.state[8].push(coordinates[2]) # th

    # Calculates the average of forwards and backwards derivatives for a given state
    def getVel(self, stateIdx):
        deriv_len = 1
        future = self.state[stateIdx].vals[0]
        past = self.state[stateIdx].vals[deriv_len]
        return (future - past)/(deriv_len * self.dt)

    # "Zeros" theta by setting current theta as the reference
    # Initialize x and y desired states to current x and y states
    def initXYT(self):
        # Clear the theta reference
        self.th_start = 0

        # Get the theta averaged over a number of samples
        num_pts = 100
        for i in range(0, num_pts):
            self.updateSensorValue()
            time.sleep(0.01)

        # Set the theta reference
        self.th_start = self.state[4].winMean()

        # Set the x and y desired
        self.state[6].push(self.state[0].winMean())
        self.state[7].push(self.state[2].winMean())

    # Print sensor values for debugging
    def printSensorVals(self):
        print("Sensor values:")
        for i in range(0,self.num_sensors):
            print("\t%d: %+0.3f, Var: %+0.3f" % (i, self.sensors[i][0].curVal(), self.sensors[i][0].winStdDev()))
       #     print("%+03.3f" % (self.sensors[i][0].curVal()), end = '| ')
       #print("%f" % (np.degrees(new_th)))

        print("\nState values:")
        for i in range(0, self.num_states):
            print("\t%d: %+0.3f" % (i, self.state[i].curVal()))

    # Calculates required motor speeds to acheive desired x, y, theta movement and sends to MCU
    def mechanumCommand(self, x, y, th):
        # Limit input range
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

        # Scale motor speeds for UART cmd and send motor control commands
        self.cmdMotor([int(x * self.MAX_PWM_CYCLES) for x in self.motorSpeeds])

    # Send motor speed values to MCU
    def cmdMotor(self, motorCmd):
        # Check that all speeds are within limits
        for cmd in motorCmd:
            if (checkValue(cmd, -MAX_PWM_CYCLES, MAX_PWM_CYCLES)):
                raise ValueError("motorCmd is not within bounds")

        # Form UART command string
        cmd = 'M ' + str(motorCmd[0]) + ' ' + str(motorCmd[1]) + ' ' \
            + str(motorCmd[2]) + ' ' + str(motorCmd[3]) + '\n'

        # Write command to serial
        self.console.writeSerialSequence([cmd])

    # Initialize data log and enable logging
    def startLog(self):
        self.data_log = 'dt (ms), Magnetometer (degrees), Rangefinder X (mm), Rangefinder Y (mm)\n'
        self.data_log_enable = True

    # Stop logging and write to file
    def stopLog(self):
        if (self.data_log_enable):
            self.data_log_enable = False
            timestr = time.strftime("%Y%m%d-%H%M%S")
            filename = 'datalog_' + timestr + '.csv'
            with open(filename, 'a') as datalog_file:
                datalog_file.write(self.data_log)

