#!/usr/local/bin/python3
"""
Simulates the robot, playfield, sensors
Units are in millimeters, degrees, and seconds (for time).
"""

import numpy as np
import helper_funcs as hf
import gym

FIELD_XMAX = 2438.4 # Maximum x dimension in mm
FIELD_YMAX = 1219.2  # Maximum y dimension in mm

MOVEMENT_SPEED = 5
TIME_STEP = 0.01 # seconds
TIME_MAX = 2 #seconds

class Box():
     def __init__(self,low,high,shape):
        self.low = low
        self.high = high
        self.shape = shape

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
    def __init__(self):
        # Set up interface with nn
        self.observation_space = gym.spaces.box.Box(low=-1.0, high=1.0, shape=(9,1),dtype=np.float32)
        self.action_space = gym.spaces.box.Box(low=-1.0, high=1.0, shape=(4,1),dtype=np.float32)

        # Robot dimensions
        self.len_x = 315.0
        self.len_y = 275.0
        self.motor_x = 127.5
        self.motor_y = 117.5

        # Magnitude and angle of a line between the center and top right corner
        self.diag_angle = np.arctan(self.len_y / self.len_x)
        self.diag_len = np.hypot(self.len_x,self.len_y) / 2

        # Magnitude and angle of a line between the center and each motor
        # in the order of front left, back left, back right, front right
        motor_angle_temp = np.arctan(self.motor_y/self.motor_x)
        self.motor_angles = [np.pi - motor_angle_temp, motor_angle_temp + np.pi,\
                -1*motor_angle_temp, motor_angle_temp]
        self.motor_radius = np.hypot(self.motor_x, self.motor_y)

        # Robot start position and orientation in the field
        self.x_start = FIELD_XMAX / 2
        self.y_start = FIELD_YMAX / 2

        # Sim vars
        self.time = 0
        self.reward = 0
        self.x = self.x_start
        self.y = self.y_start
        self.theta = 0
        self.terminal = 0

        # Converts wheel velocities to tranlational x,y, and rotational velocities
        self.max_wheel_w = 2*np.pi*4 # max wheel rotational velocity in rad/s
        r =  30.0   # Wheel radius in mm
        L1 = 119.35 # Half the distance between the centers of front and back wheels in mm
        L2 = 125.7  # Half the distance between the centers of left and right wheels in mm
        self.mecanum_xfer = (r/4) * np.array([[1,1,1,1],[-1,1,-1,1],\
                [1/(L1+L2),-1/(L1+L2),-1/(L1+L2),1/(L1+L2)]])

        # Initialize rangefinders
        self.rf1 = SimRangefinder(self, 60.0, self.len_y/2.0, 90.0)
        self.rf2 = SimRangefinder(self, -1 * self.len_x/2.0, -40.0, 180.0)
        self.rf3 = SimRangefinder(self, 0.0, -1 * self.len_y/2.0, 270.0)
        self.rf4 = SimRangefinder(self, self.len_x/2.0, -40.0, 0.0)
        self.imu = SimIMU()

        # Initialize sim vars
        self.reset()

    def reset(self):
        # Reset vars
        self.time = 0
        self.reward = 0
        self.x = self.x_start + np.random.normal(0, 100)
        self.y = self.y_start + np.random.normal(0, 100)
        self.theta = np.random.normal(0, 3)
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
        # Calculate motor voltages for the specified control inputs
        # ctrl[0]: Desired robot speed, -1 to 1
        # ctrl[1]: Desired translation angle, -1 to 1, +1 dead right, 0.5 dead up, 0 dead left, -0.5 dead down
        # ctrl[2]: Desired rotational speed, -1 to 1, +1 CW, -1 CCW
        vd = ctrl[0]
        theta_d = np.pi * (ctrl[1] + 1)
        v_theta = ctrl[2]
        v = np.zeros([4,1])
        v[2][0] = vd * np.sin(theta_d + np.pi/4) + v_theta
        v[3][0] = vd * np.cos(theta_d + np.pi/4) - v_theta
        v[1][0] = vd * np.cos(theta_d + np.pi/4) + v_theta
        v[0][0] = vd * np.sin(theta_d + np.pi/4) - v_theta

        self.time += TIME_STEP
        self.action = ctrl

         # Wheel rotational velocities in degrees/sec
        #wheel_w = self.max_wheel_w * np.array([[ctrl[0]],[ctrl[1]],[ctrl[2]],[ctrl[3]]])
        wheel_w = self.max_wheel_w * v

        # Calculate robot frontwards, rightwards, and rotational velocities
        velocities = np.matmul(self.mecanum_xfer, wheel_w)
        self.right_vel = velocities[0][0]
        self.front_vel = velocities[1][0]
        self.rotation_vel = velocities[2][0]

        # Calculate the x and y velocity components
        self.theta = (self.theta + (self.rotation_vel * TIME_STEP * 180 / np.pi))
        self.x_vel = self.right_vel * np.cos(np.radians(self.theta)) + self.front_vel * np.sin(np.radians(self.theta))
        self.y_vel = self.right_vel * np.sin(np.radians(self.theta)) + self.front_vel * np.cos(np.radians(self.theta))

        # Calculate cos and sin of the angle
        angle = np.radians(self.theta % 180)
        cosval = np.cos(angle)

        # Calculate the x and y spacing for collision detection
        if (cosval >= 0):
            x_space = abs(self.diag_len * np.cos(np.pi - self.diag_angle + angle))
            y_space = abs(self.diag_len * np.sin(self.diag_angle + angle))
        else:
            x_space = abs(self.diag_len * np.cos(self.diag_angle + angle))
            y_space = abs(self.diag_len * np.sin(np.pi - self.diag_angle + angle))

        # Assign new x,y location
        self.x = hf.limitValue(self.x + self.x_vel * TIME_STEP,x_space,FIELD_XMAX - x_space)
        self.y = hf.limitValue(self.y + self.y_vel * TIME_STEP,y_space,FIELD_YMAX - y_space)

        # Update sensor measurements
        self.state = self.updateSensors()

        # Get reward from executing the action
        self.reward = self.updateReward()

        # End of sim
        if (self.time > TIME_MAX):
            self.terminal = 1

        info = None

        return self.state, self.reward, self.terminal, info

    def getReward(self):
        return self.reward

    def updateReward(self):      
        # Normalize theta to +/- pi
        theta = np.radians(self.theta)
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        
        # Keep angle at 0 with minimal rotation velocity
        self.reward_theta = -1.0 * pow(theta,2) / 2

        # Minimize effort
        self.reward_effort = 0
        try:
            for action in self.action:
                self.reward_effort -= action**2
        except: 
            pass

        # Rewarded for staying near center of field
        distance_from_center = np.hypot(self.x,self.y)
        self.reward_dist =  -1.0 * (distance_from_center / 1000.0)
        
        # Minimize velocities
        self.reward_vel = -1.0 * (0.00001 * (pow(self.front_vel,2) + pow(self.right_vel,2) + pow(self.rotation_vel,2)))

        self.reward = self.reward_theta + self.reward_effort + self.reward_dist + self.reward_vel
        return self.reward

    def getState(self):
        return self.state

    def updateSensors(self):
        self.rf1.getMeas(self)
        self.rf2.getMeas(self)
        self.rf3.getMeas(self)
        self.rf4.getMeas(self)
        self.imu.getMeas(self)
        self.state = [self.rf1.meas, self.rf2.meas, self.rf3.meas, self.rf4.meas, 
                      self.imu.cos, self.imu.sin,
                      self.front_vel,self.right_vel, self.rotation_vel]
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
        self.position_theta = np.arctan2(y,x)
        self.radius = np.hypot(x,y)
        rf_field_theta = np.radians(robot.theta) + self.position_theta
        self.dim = [robot.x + self.radius * np.cos(rf_field_theta),\
            robot.y + self.radius * np.sin(rf_field_theta), 0.0, 0.0]
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
        rf_field_theta = np.radians(robot.theta) + self.position_theta
        x1 = robot.x + self.radius * np.cos(rf_field_theta)
        y1 = robot.y + self.radius * np.sin(rf_field_theta)

        # Calculate cos and sin of the angle of where the rangefinder is oriented
        rf_aim_theta = np.radians(robot.theta + self.theta)
        cosval = np.cos(rf_aim_theta)
        sinval = np.sin(rf_aim_theta)

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
    def __init__(self,meas_period = 0.01, sigma = 0.0):
        self.timebank = 0
        self.cos = 0
        self.sin = 0
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

        # Add noise to measurement, convert to radians
        self.meas = (robot.theta + np.random.normal(0, self.sigma)) % 360.0
        theta = np.radians(self.meas)

        # Get x/y components of theta
        self.cos = np.cos(theta)
        self.sin = np.sin(theta)


class SimMicroSW():
    def __init__(self):
        return

