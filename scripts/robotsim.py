#!/usr/local/bin/python3
"""
Simulates the robot, playfield, sensors
Units are in millimeters, degrees, and seconds (for time).
"""

import numpy as np
import math
import helper_funcs as hf
import gym

FIELD_XMAX = 2438.4 # Maximum x dimension in mm
FIELD_YMAX = 1219.2  # Maximum y dimension in mm

MOVEMENT_SPEED = 5
TIME_STEP = 0.01 # seconds
TIME_MAX = 1 #seconds


class SimRobot():
    """ Simulates a robot with sensors. Robot coordinates are defined as cartesian
    xy, origin at robot's center. +y is directly towards front side of robot and +x is
    directly towards right side of robot. Theta is the angle of a radial vector where 0 degrees is
    pointing directly towards the right side of the robot and 90 degrees is directly towards the front.
    len_x = length of the robot left to right
    len_y = length of the robot front to back
    x: starting x position of the robot origin in field coordinates
    y: starting y position of the robot origin in field coordinates
    theta: starting angle of the robot origin in field coordinates
    """
    def __init__(self,len_x=315.0,len_y=275.0,theta=0.0):
        # Set up interface with nn
        self.observation_space = gym.spaces.box.Box(low=-1.0, high=1.0, shape=(5,1))
        self.action_space = gym.spaces.box.Box(low=-1.0, high=1.0, shape=(5,1))

        # Robot dimensions
        self.len_x = len_x
        self.len_y = len_y

        # Magnitude and angle of a line between the center and top right corner
        self.diag_angle = math.atan(len_y / len_x)
        self.diag_len = math.sqrt(pow(len_x,2) + pow(len_y,2)) / 2

        # Robot position and orientation in the field
        self.x_start = FIELD_XMAX / 2
        self.y_start = FIELD_YMAX / 2
        self.theta = theta

        # Sim vars
        self.time = 0
        self.reward = 0
        self.x = self.x_start
        self.y = self.y_start
        self.terminal = 0

        # Converts wheel velocities to tranlational x,y, and rotational velocities
        self.max_wheel_w = 2*math.pi*4 # max wheel rotational velocity in rad/s
        r =  30.0   # Wheel radius in mm
        L1 = 119.35 # Half the distance between the centers of front and back wheels in mm
        L2 = 125.7  # Half the distance between the centers of left and right wheels in mm
        self.mecanum_xfer = (r/4) * np.array([[1,1,1,1],[-1,1,-1,1],\
                [1/(L1+L2),-1/(L1+L2),-1/(L1+L2),1/(L1+L2)]])

        # Initialize rangefinders
        self.rf1 = SimRangefinder(self, 60.0, self.len_y/2.0, 90.0)
        self.rf2 = SimRangefinder(self, -1 * self.len_x/2.0, -40.0, 180.0)
        self.rf3 = SimRangefinder(self, 0.0, -1 * len_y/2.0, 270.0)
        self.rf4 = SimRangefinder(self, self.len_x/2.0, -40.0, 0.0)
        self.imu = SimIMU()

        # Initialize sim vars
        self.reset()

    def reset(self):
        # Reset vars
        self.time = 0
        self.reward = 0
        self.x = self.x_start
        self.y = self.y_start
        self.terminal = 0
        self.step([0,0,0,0,0])
        return self.state

    def seed(self, seed):
        self.seed = seed

    def step(self,ctrl):
        """ ctrl is a array with the various motor/launcher control inputs, valid range -1 to +1
        ctrl[0]: Front left motor, +1 rotates towards right
        ctrl[1]: Back left motor, +1 rotates towards right
        ctrl[2]: Back right motor, +1 rotates towards right
        ctrl[3]: Front right motor, +1 rotates towards right
        ctrl[4]: Launcher
        """
        self.time += TIME_STEP

         # Wheel rotational velocities in degrees/sec
        wheel_w = self.max_wheel_w * np.array([[ctrl[0]],[ctrl[1]],[ctrl[2]],[ctrl[3]]])

        # Calculate robot frontwards, rightwards, and rotational velocities
        velocities = np.matmul(self.mecanum_xfer, wheel_w)
        right_vel = velocities[0][0]
        front_vel = velocities[1][0]
        rotation_vel = velocities[2][0]

        # Calculate the x and y velocity components
        self.theta = (self.theta + (rotation_vel * TIME_STEP * 180 / math.pi)) % 360.0
        x_vel = right_vel * math.cos(math.radians(self.theta)) + front_vel * math.sin(math.radians(self.theta))
        y_vel = right_vel * math.sin(math.radians(self.theta)) + front_vel * math.cos(math.radians(self.theta))

        # Calculate cos and sin of the angle
        angle = math.radians(self.theta % 180)
        cosval = math.cos(angle)

        # Calculate the x and y spacing for collision detection
        if (cosval >= 0):
            x_space = abs(self.diag_len * math.cos(math.pi - self.diag_angle + angle))
            y_space = abs(self.diag_len * math.sin(self.diag_angle + angle))
        else:
            x_space = abs(self.diag_len * math.cos(self.diag_angle + angle))
            y_space = abs(self.diag_len * math.sin(math.pi - self.diag_angle + angle))

        # Assign new x,y location
        self.x = hf.limitValue(self.x + x_vel * TIME_STEP,x_space,FIELD_XMAX - x_space)
        self.y = hf.limitValue(self.y + y_vel * TIME_STEP,y_space,FIELD_YMAX - y_space)

        # Update sensor measurements
        self.state = self.updateSensors()
        self.reward = self.updateReward()

        # End of sim
        self.terminal = self.time > TIME_MAX
        info = None

        return self.state, self.reward, self.terminal, info

    def getReward(self):
        return self.reward

    def updateReward(self):
        self.reward = 1000 - math.sqrt(pow(self.x-FIELD_XMAX/2,2) + pow(self.y - FIELD_YMAX/2,2))
        return self.reward

        return self.reward

    def getState(self):
        return self.state

    def updateSensors(self):
        self.rf1.getMeas(self)
        self.rf2.getMeas(self)
        self.rf3.getMeas(self)
        self.rf4.getMeas(self)
        self.imu.getMeas(self)
        self.state = [self.rf1.meas, self.rf2.meas, self.rf3.meas, self.rf4.meas, self.imu.meas]
        return self.state


class SimRangefinder():
    """ Simulates a rangefinder.
    x: x-position in robot coordinates
    y: y-position in robot coordinates
    theta: orientation in robot coordinates
    min_range: minimum measurement range
    max_range: maximum measurement range
    meas_period: amount of time between measurements
    """
    def __init__(self,robot,x,y,theta,max_range=1200.0,meas_period=0.033):
        self.theta = theta
        self.position_theta = math.atan2(y,x)
        self.radius = math.sqrt(math.pow(x,2) + math.pow(y,2))
        rf_field_theta = math.radians(robot.theta) + self.position_theta
        self.dim = [robot.x + self.radius * math.cos(rf_field_theta),\
            robot.y + self.radius * math.sin(rf_field_theta), 0.0, 0.0]
        self.max_range = max_range
        self.timebank = 0
        self.meas_period = meas_period
        self.meas = 0

    def getMeas(self, robot):
        """ Returns the measurement of the rangefinder in mm.
        robot: robot object
        """
        # Add some available time to the timebank
        self.timebank += TIME_STEP
        # Only make a measurement if there's enough time in the bank
        if (self.timebank < self.meas_period):
            return
        # Use up some time from the timebank
        self.timebank -= self.meas_period

        # Calculate the sensor's position and orientation in the field
        # rf_field_theta is used to calculate where the sensor is but not where its oriented
        rf_field_theta = math.radians(robot.theta) + self.position_theta
        x1 = robot.x + self.radius * math.cos(rf_field_theta)
        y1 = robot.y + self.radius * math.sin(rf_field_theta)

        # Calculate cos and sin of the angle of where the rangefinder is oriented
        rf_aim_theta = math.radians(robot.theta + self.theta)
        cosval = math.cos(rf_aim_theta)
        sinval = math.sin(rf_aim_theta)

        if cosval >= 0:
            meas_x = FIELD_XMAX - x1
        else:
            meas_x = -1 * x1
        if sinval >= 0:
            meas_y = FIELD_YMAX - y1
        else:
            meas_y = -1 * y1

        # Calculate the two possible measurements
        try:
            hx = meas_x / cosval # If the hypotenuse extends to a vertical wall
            hy = meas_y / sinval # If the hypotenuse extends to a horizontal wall

            # The correct measurement is the shorter one
            if (hx < hy):
                meas = hx
                x2 = x1 + meas_x
                y2 = y1 + meas * sinval
            else:
                meas = hy
                x2 = x1 + meas * cosval
                y2 = y1 + meas_y
        except ZeroDivisionError:
            if (cosval != 0):
                meas = meas_x / cosval
                x2 = x1 + meas_x
                y2 = y1 + meas * sinval
            else:
                meas = meas_y / sinval
                x2 = x1 + meas * cosval
                y2 = y1 + meas_y

        # Add noise to measurement
        sigma = 2E-05 * pow(meas,2) - 0.0062 * meas + 2.435
        meas += np.random.normal(0, sigma)

        # Only update if measurement is in range
        if ((meas > 30) and (meas < 1200)):
            self.dim = [x1,y1,x2,y2]
            self.meas = meas


class SimIMU():
    def __init__(self,meas_period = 0.01, sigma = 0.1):
        self.timebank = 0
        self.meas = 0
        self.meas_period = meas_period
        self.sigma = sigma

    def getMeas(self, robot):
        # Add some available time to the timebank
        self.timebank += TIME_STEP
        # Only make a measurement if there's enough time in the bank
        if (self.timebank < self.meas_period):
            return
        # Use up some time from the timebank
        self.timebank -= self.meas_period

        # Add noise to measurement
        self.meas = (robot.theta + np.random.normal(0, self.sigma)) % 360


class SimMicroSW():
    def __init__(self):
        return

