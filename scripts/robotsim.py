#!/usr/local/bin/python3
"""
Simulates the robot, playfield, sensors
Units are in millimeters, degrees, and seconds (for time).

The MIT License (MIT)

Copyright (c) 2018 Justin Ng

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from robot import *
from sensors import *
import numpy as np
import helper_funcs as hf
import gym
from gym.utils import seeding
from os import path
import pyglet
from pyglet.gl import *
from ann import ann
import time
from gym.envs.classic_control import rendering

DEBUG_PRINT = False

# Field and robot dimensional constants
FIELD_XMAX = Robot.FIELD_XMAX
FIELD_YMAX = Robot.FIELD_YMAX
LEN_X = Robot.LEN_X
LEN_Y = Robot.LEN_Y

# Simulates motor variance. Controls percentage of how much motors differ.
# Ex: 0.05 means motor friction and voltage sensitivity parameters will vary up to 5%
MOTOR_DELTA = 0.0

# Maximum simulation time in seconds
TIME_MAX = 10

# Stores observation parameters, min/max value and shape
class Box():
     def __init__(self,low,high,shape):
        self.low = low
        self.high = high
        self.shape = shape

class SimRobot():
    """ Simulates a robot with sensors. Robot coordinates are defined as cartesian
    xy, origin at robot's center. +y is directly towards front side of robot and +x is
    directly towards right side of robot. Theta is the angle of a radial vector where -90 degrees is
    pointing directly towards the right side of the robot and 0 degrees is directly towards the front.
    train: 'angle', 'translation', or '' to choose which setup to train
    state_start: starting state of robot with 9 elements
        0: x position
        1: x velocity
        2: y position
        3: y velocity
        4: rotational position theta
        5: rotational velocity
        6: desired x position
        7: desired y position
        8: desired th position
    """


    def __init__(self, train = None, online = False,
            state_start = [FIELD_XMAX/2, 0, FIELD_YMAX/2, 0, 0, 0, FIELD_XMAX/2, FIELD_YMAX/2, 0]):
        # For rendering
        self.viewer = None
        self.train = train
        self.online = online

        # Used for online training. Causes reset behavior to switch directions on each run
        # to prevent robot from running out of cable or twisting it excessively
        self.last_reset_dir = 1

        # Tracks maximum cycle time to ensure not exceeding dt requirement
        self.max_cyc_time = 0

        # Length of control input u
        self.u_len = 3  # X, Y, theta

        # Cycle time step
        self.dt = Robot.dt

        # Instantiate real robot if online training
        self.robot = Robot(0)

        # Connect to robot
        if (self.online):
            if self.robot.openSerial():
                print("Failed to connect to robot. Quitting.")
                quit()

            print("Initializing desired x, y, theta...")
            self.robot.initXYT()
        else:
        # Simulated robot calculations
            # Structures/parameters for simulating robot
            self.state_start = np.array(state_start)

            # Magnitude and angle of a line between the center and top right corner
            self.diag_angle = np.arctan(LEN_Y / LEN_X)
            self.diag_len = np.hypot(LEN_X,LEN_Y) / 2
            self.en_wall_collision = False

            # Track wheel velocities and accelerations
            self.wheel_thetadotdot =  np.zeros(4) # Wheel rotational acceleration in rad/s/2
            self.wheel_thetadot = np.zeros(4)     # Wheel rotational velocities in rad/s

            # Mecanum transfer function converts wheel velocities to tranlational x,y, and rotational velocities
            r = Robot.WHEEL_R
            L1 = Robot.FRONT_BACK_WHEEL_DIST
            L2 = Robot.LEFT_RIGHT_WHEEL_DIST
            self.mecanum_xfer = (r/4) * np.array([[1,1,1,1],[-1,1,-1,1],\
                    [1/(L1+L2),-1/(L1+L2),-1/(L1+L2),1/(L1+L2)]])

        # Set up action space and observation space. Should match robot.py self.sensors list
        self.sensors = []                                                      # Sensor index
        self.sensors.append(SimRangefinder(60.0, LEN_Y/2.0, 90.0, self.dt, FIELD_XMAX, FIELD_YMAX))         # 0
        self.sensors.append(SimRangefinder(-1 * LEN_X/2.0, -40.0, 180.0, self.dt, FIELD_XMAX, FIELD_YMAX))  # 1
        self.sensors.append(SimRangefinder(0.0, -1 * LEN_Y/2.0, 270.0, self.dt, FIELD_XMAX, FIELD_YMAX))    # 2
        self.sensors.append(SimRangefinder(LEN_X/2.0, -40.0, 0.0, self.dt, FIELD_XMAX, FIELD_YMAX))         # 3
        self.sensors.append(SimIMU(self.dt, 'magx'))                                                        # 4
        self.sensors.append(SimIMU(self.dt, 'magy'))                                                        # 5
        self.sensors.append(SimIMU(self.dt, 'magz'))                                                        # 6
        self.sensors.append(SimIMU(self.dt, 'accx'))                                                        # 7
        self.sensors.append(SimIMU(self.dt, 'accy'))                                                        # 8
        self.sensors.append(SimIMU(self.dt, 'accz'))                                                        # 9

        # Add sensors and number of action vars depending on what network is being trained
        if (train == 'angle'):
            act_dim = 1
            self.net_index = 0
            obs_high = np.array([1., 1., 25.])

        elif (train == 'transx'):
            act_dim = 1
            self.net_index = 1
            obs_high = np.array([2400., 1600.])

        elif (train == 'transy'):
            act_dim = 1
            self.net_index = 2
            obs_high = np.array([2400., 1600.])

        elif (train == 'all'):
            act_dim = 3
            self.net_index = 3
            obs_high = np.array([2400., 1600., 2400., 1600., 1., 1., 25.])

        elif (train == 'test_three_actor'):
            act_dim = 3
            self.net_index = 3
            obs_high = np.array([2400., 1600., 2400., 1600., 1., 1., 25.])

        elif (train == 'sim'):
            act_dim = 3
            self.net_index = 4
            #obs_high = [2400., 1600.]

        # Setup observation space
        self.observation_space = gym.spaces.box.Box(low=-obs_high, high=obs_high)

        # Maximum action value and shape
        self.action_space = gym.spaces.box.Box(low=-2.0, high=2.0, shape=(act_dim,))

        # Initialize sim vars by running a reset w/o randomization
        self.reset(False)

    def setWallCollision(self, mode):
        if (mode == True):
            self.en_wall_collision = True
        elif (mode == False):
            self.en_wall_collision = False
        else:
            raise ValueError("Bad value for mode in setWallCollision.")

    def reset(self, randomize=True, testNet=None, step=0, step_max=50):
        # Reset vars
        self.time = 0
        self.reward = 0
        self.terminal = 0
        self.last_u = np.zeros(self.u_len)
        self.testNet = testNet

        # Limit range of values for each state var
        if (self.train == 'angle'):
            max_val = np.pi
            min_val = -np.pi
            state_index = 4
        elif (self.train == 'transx'):
            max_val = FIELD_XMAX - LEN_X / 2
            min_val = LEN_X / 2
            state_index = 0
        elif (self.train == 'transy'):
            max_val = FIELD_YMAX - LEN_Y / 2
            min_val = LEN_Y / 2
            state_index = 2
        elif (self.train == 'all'):
            max_val = np.array([FIELD_XMAX - LEN_X / 2, FIELD_YMAX - LEN_Y / 2, np.pi])
            min_val = np.array([LEN_X / 2, LEN_Y / 2, -np.pi])
            state_index = [0,2,4]

        # Initialize to starting state
        self.state = self.state_start

        if (randomize):
            if (self.online):
                # Set a new desired x, y, theta
                xdes = np.random.uniform(low=40, high=1000)
                ydes = np.random.uniform(low=40, high=1000)
                thdes = np.random.uniform(low=-np.pi, high=np.pi)
                self.robot.setDesired([xdes, ydes, thdes])
            else:
                self.state[state_index] = np.random.uniform(low=min_val, high=max_val)

                # Randomize variable friction and voltage sensitivity per wheel
                self.friction_var = np.array([Robot.friction_constant * \
                        (1+np.random.uniform(-MOTOR_DELTA, MOTOR_DELTA)) for i in range(4)])
                self.v_sensitivity = np.array([1 + np.random.uniform(-MOTOR_DELTA, MOTOR_DELTA) \
                        for i in range(4)])
        elif (testNet):
            val = (max_val - min_val) * step / step_max + min_val
            self.state[state_index] = val
        else:
            self.friction_var = np.array([Robot.friction_constant for i in range(4)])
            self.v_sensitivity = np.array([1 for i in range(4)])

        # Update desired state
        self.robot.setDesired(self.state[6:9])

        # Update observations
        for i in range(0,3):
            sensor_vals = self.getSensorVals()
            self.updateSensors(sensor_vals)
            self.updateState()
            self.updateObservation()

        # Keep track of time for cycle timing
        if (self.online):
            self.last_time = self.robot.getTime()

        return self.obs

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self,u):
        # Determine the control input array u depending on what's being trained
        # u[0]: Field x velocity
        # u[1]: Field y velocity
        # u[2]: Rotational speed, -1 to 1, +1 CW, -1 CCW

        if self.train == 'sim':
            u_transx = [u[0]]
            u_transy = [u[1]]
            u_angle = [u[2]]

        elif self.train == 'angle':
            u_transx = [0]
            u_transy = [0]
            u_angle = u

        elif self.train == 'transx':
            u_transx = u
            u_transy = [0] #self.transy_ann.predict(self.obs_sets[2])
            u_angle = [0] #self.angle_ann.predict(np.reshape(self.obs_sets[0],(1,3)))[0]

        elif self.train == 'transy':
            u_transx = [0] # self.transx_ann.predict(np.reshape(self.obs_sets[1],(1,2)))[0]
            u_transy = u
            u_angle = [0] # self.angle_ann.predict(np.reshape(self.obs_sets[0],(1,3)))[0]

        elif self.train == 'all':
            u_transx = [u[0]]
            u_transy = [u[1]]
            u_angle = [u[2]]

        u = np.concatenate((u_transx, u_transy, u_angle))
        u = np.clip(u, -2., 2.)

        dt = self.dt
        self.time += dt

        # Execute command
        self.execute(u)

        # Get sensor vals
        sensor_vals = self.getSensorVals()

        # Update sensor vars
        self.updateSensors(sensor_vals)

        # Update state
        self.updateState()

        if (self.online):
            # Delay excess time in cycle to make all cycles equal time
            cur_time = self.robot.getTime()
            while ((cur_time - self.last_time) < dt):
                cur_time = self.robot.getTime()
            #cyc_time = cur_time - self.last_time
            #if (cyc_time > self.max_cyc_time):
            #    self.max_cyc_time = cyc_time
                #print("MAX Tcyc %0.6f" % self.max_cyc_time)
            #print("Tcyc %0.6f" % cyc_time)
            self.last_time = cur_time

        # Update last u
        self.last_u = u

        # Update observations
        self.updateObservation()

        # Get reward from executing the action
        if (self.train == 'sim'):
            pass
        else:
            self.updateReward()
        #time.sleep(0.2)

        if (DEBUG_PRINT):
            self.robot.printSensorVals()

        # End cycle if time hits max or robot moves to an extreme bound
        error_x = abs(self.state[0] - self.state[6])
        error_y = abs(self.state[2] - self.state[7])
        end_early = ((self.testNet == None) and
            ((error_x > 1200 - self.robot.LEN_X/2) or
            (error_y > 600 - self.robot.LEN_Y/2)))

        if ((self.time > TIME_MAX) or end_early):# or (self.online and (abs(self.state[4]) > 1.2))):
            self.halt()
            self.terminal = 1

        info = {}
        return self.obs, self.reward, self.terminal, info

    def execute(self, u):
        if (self.online):
            self.robot.execute(u)
        else:
            # Get state vars
            x, xdot, y, ydot, th, thdot, xdes, ydes, thdes = self.state
            dt = self.dt

            #  Trajectory control inputs
            v_theta = u[2]
            vd = np.hypot(u[0],u[1]) / 2.0
            #print("%f | %f" % (u[1],vd))
             # Adjust desired translation angle to field coordinates
            theta_d = np.arctan2(u[1],u[0]) - self.robot.state[4].curVal()

            # Calculate voltage ratios for each motor to acheive desired trajectory
            v = np.zeros(4)
            v[2] = vd * np.sin(theta_d + np.pi/4) + v_theta
            v[3] = vd * np.cos(theta_d + np.pi/4) - v_theta
            v[1] = vd * np.cos(theta_d + np.pi/4) + v_theta
            v[0] = vd * np.sin(theta_d + np.pi/4) - v_theta

            # Normalize ratios to 1 if the maxval is > 1
            maxval = np.amax(np.absolute(v))
            v = v / maxval if (maxval > 1) else v

            # Convert voltage ratios to wheel rotational velocities in rad/sec
            # Model linear relationship between voltage and acceleration. Friction linear with velocity.
            self.wheel_thetadotdot = np.multiply(self.v_sensitivity, v) * Robot.wheel_max_thetadotdot - \
                    np.multiply(self.wheel_thetadot, self.friction_var)
            self.wheel_thetadot = self.wheel_thetadot + self.wheel_thetadotdot * dt

            # Calculate robot frontwards, rightwards, and rotational velocities
            velocities = (np.matmul(self.mecanum_xfer, (self.wheel_thetadot[np.newaxis]).T)).T[0]
            rightdot = velocities[0]
            frontdot = velocities[1]
            newthdot = velocities[2]

            # Calculate the new theta
            newth = th + newthdot*dt

            # Calcuate x and y velocity components
            newxdot = rightdot * np.cos(newth) + frontdot * np.sin(newth)
            newydot = rightdot * np.sin(newth) + frontdot * np.cos(newth)

            if (self.en_wall_collision == True):
                # Calculate cos and sin of the angle
                angle = newth % np.pi
                cosval = np.cos(angle)

                # Calculate the x and y spacing for collision detection
                if (cosval >= 0):
                    x_space = abs(self.diag_len * np.cos(np.pi - self.diag_angle + angle))
                    y_space = abs(self.diag_len * np.sin(self.diag_angle + angle))
                else:
                    x_space = abs(self.diag_len * np.cos(self.diag_angle + angle))
                    y_space = abs(self.diag_len * np.sin(np.pi - self.diag_angle + angle))

                # Assign new x,y location
                newx = np.clip(x + newxdot * dt, x_space - 0, FIELD_XMAX - x_space + 0)
                newy = np.clip(y + newydot * dt, y_space - 0, FIELD_YMAX - y_space + 0)
            else:
                # Assign new x,y location
                newx = x + newxdot * dt
                newy = y + newydot * dt

            # Determine new desired location
            newxdes = xdes
            newydes = ydes
            newthdes = thdes

            # Determine new state
            self.state = np.array([newx, newxdot, newy, newydot, newth, newthdot, newxdes, newydes, newthdes])

    # Get sensor values but don't store them into self.robot yet
    def getSensorVals(self):
        if (self.online):
            sensor_vals = self.robot.getSensorVals()
        else:
            sensor_vals = []
            for sensor in self.sensors:
                sensor.update(self.state)
                sensor_vals.append(sensor.meas)
        return sensor_vals

    def updateSensors(self, sensor_vals):
        self.robot.updateSensors(sensor_vals)

    def updateState(self):
        self.robot.updateState()

    def updateStateOnline(self):
        # Update sensor observation and state array
        self.robot.updateSensorValue()
        new_state = []
        for i in range(0,9):
            new_state.append(self.robot.state[i].curVal())
        self.state = new_state

    def updateObservation(self):
        self.obs_sets = self.robot.getObservation()
        self.obs = self.obs_sets[self.net_index]

    def updateReward(self):
        x = self.robot.state[0].curVal()
        xdot = self.robot.state[1].curVal()
        y = self.robot.state[2].curVal()
        ydot = self.robot.state[3].curVal()
        th = self.robot.state[4].curVal()
        thdot = self.robot.state[5].curVal()
        xdes = self.robot.state[6].curVal()
        ydes = self.robot.state[7].curVal()
        thdes = self.robot.state[8].curVal()

        error_x = abs(x - self.state[6])
        error_y = abs(self.state[2] - self.state[7])
        end_early = ((self.testNet == None) and
            ((error_x > 1200 - self.robot.LEN_X/2) or
            (error_y > 600 - self.robot.LEN_Y/2)))

        # If ending early, receive penalty
        if (end_early):
            penalty = -100.0
        else:
            penalty = 0

        # Rewarded for staying near desired x coordinate
        r_x = -0.00001 * (x - xdes)**2.
        # Minimize velocities
        r_xdot =  -0.0000005 * xdot**2
        # Minimize effort
        r_ux = -0.001 * self.last_u[0]**2

        # Rewarded for staying near desired y coordinate
        r_y =  -0.00001 * (y - ydes)**2.
        # Minimize velocities
        r_ydot = -0.0000005 * ydot**2
        # Minimize effort
        r_uy = -0.001 * self.last_u[1]**2

        # Keep angle at 0
        r_th = -1.0 * (self.angle_normalize(th) - self.angle_normalize(thdes))**2.
        # Minimize velocities
        r_thdot = -0.1 * thdot**2
        # Minimize effort
        r_uth = -0.001 * self.last_u[2]**2

        # Calc rewards for each case
        reward_x  = r_x  + r_xdot  + r_ux
        reward_y  = r_y  + r_ydot  + r_uy
        reward_th = r_th + r_thdot + r_uth
        # print("%0.3f %0.3f %0.3f" % (reward_x, reward_y, reward_th))

        if (self.train == 'angle'):
            self.reward = reward_th
        elif (self.train == 'transx'):
            self.reward = reward_x
        elif (self.train == 'transy'):
            self.reward = reward_y
        elif (self.train == 'all'):
            self.reward = reward_x + reward_y + reward_th

        if (DEBUG_PRINT):
            print("Reward: %f" % self.reward)

        # Additional penalty for ending early
        self.reward += penalty
        return self.reward

    def render(self, mode='human'):
        if self.viewer is None:
            self.viewer = rendering.Viewer(1300,650)
            self.viewer.set_bounds(0,FIELD_XMAX,0,FIELD_YMAX)

            # Robot image
            fname = path.join(path.dirname(__file__), "images/robot.png")
            self.img_robot = rendering.Image(fname, LEN_X, LEN_Y)
            self.img_robot.set_color(1,1,1)
            self.imgtrans_robot = rendering.Transform()
            self.img_robot.add_attr(self.imgtrans_robot)
            self.viewer.add_geom(self.img_robot)

            # Rotation direction indicator
            fname = path.join(path.dirname(__file__), "images/clockwise.png")
            self.img_rot_ind = rendering.Image(fname, LEN_X, LEN_X)
            #self.img_robot.set_color(0,0,0)
            self.imgtrans_rot_ind = rendering.Transform()
            self.img_rot_ind.add_attr(self.imgtrans_rot_ind)
            self.viewer.add_geom(self.img_rot_ind)

        # Robot
        self.imgtrans_robot.set_translation(self.state[0], self.state[2])
        self.imgtrans_robot.set_rotation(self.state[4])

        # Motion indicators
        self.imgtrans_rot_ind.scale = (self.last_u[2]/2, np.abs(self.last_u[2])/2)
        self.imgtrans_rot_ind.set_translation(self.state[0], self.state[2])
        self.draw_radial_arrow(self.state[0], self.state[2], self.state[1], self.state[3])

        # Rangefinder lines
        for sensor in self.sensors:
            if (sensor.type == 'Rangefinder'):
                self.viewer.draw_line(sensor.dim[0], sensor.dim[1])

        # Desired location indicator
        self.viewer.draw_line([self.state[0], self.state[2]], [self.state[6], self.state[7]])

        return self.viewer.render(return_rgb_array = mode=='rgb_array')

    def draw_radial_arrow(self,x,y,xdot,ydot):
        mag = np.hypot(xdot, ydot)
        theta = np.arctan2(ydot,xdot)
        x2 = x + 1 * mag * np.cos(theta)
        y2 = y + 1 * mag * np.sin(theta)
        self.viewer.draw_line([x, y], [x2, y2])
        #self.draw_center_triangle_filled(x2,y2,10 if mag > 0 else -10,theta)

    # Draws an equilateral triangle centered at x,y with a vertex at angle theta
    def draw_center_triangle_filled(self,x,y,mag,theta):
        x1 = x + mag * np.cos(theta)
        y1 = y + mag * np.sin(theta)
        x2 = x + mag * np.cos(theta + 2*np.pi/3)
        y2 = y + mag * np.sin(theta + 2*np.pi/3)
        x3 = x + mag * np.cos(theta + 4*np.pi/3)
        y3 = y + mag * np.sin(theta + 4*np.pi/3)
        self.viewer.draw_polygon(((x1, y1), (x2, y2), (x3, y3)))

    def angle_normalize(self, x):
        return (((x+np.pi) % (2*np.pi)) - np.pi)

    def halt(self):
        if (self.online):
            self.robot.halt()
