#!/usr/local/bin/python3
"""
Simulates the robot, playfield, sensors
Units are in millimeters, degrees, and seconds (for time).
"""

import numpy as np
import helper_funcs as hf
import gym
from gym.utils import seeding

FIELD_XMAX = 2438.4 # Maximum x dimension in mm
FIELD_YMAX = 1219.2  # Maximum y dimension in mm

TIME_MAX = 10 #seconds

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
    len_x = length of the robot left to right
    len_y = length of the robot front to back
    x: starting x position of the robot origin in field coordinates
    y: starting y position of the robot origin in field coordinates
    theta: starting angle of the robot origin in field coordinates
    """
    def __init__(self, state_start = [FIELD_XMAX/2, 0, FIELD_YMAX/2, 0, 0, 0]):
        # Set up interface with nn
        high = np.array([1., 1., 8.])
        self.action_space = gym.spaces.box.Box(low=-2.0, high=2.0, shape=(1,))
        self.observation_space = gym.spaces.box.Box(low=-high, high=high)

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
        xdot_start = 0
        ydot_start = 0
        self.state_start = np.array(state_start)
            
        # Sim vars
        self.dt = 0.05
        self.time = 0
        self.reward = 0
        self.terminal = 0

        # Mecanum equation converts wheel velocities to tranlational x,y, and rotational velocities
        self.friction_constant = 5                                      # Friction constant in 1/s
        self.wheel_max_thetadotdot = 430                                # Max wheel rotational acceleration in rad/s^2
        self.wheel_max_thetadot = 24.46                                 # Max wheel rotational velocity in rad/s
        self.wheel_thetadotdot =  np.zeros([4,1])                       # Wheel rotational acceleration in rad/s/2
        self.wheel_thetadot = np.zeros([4,1])                           # Wheel rotational velocities in rad/s
        r =  30.0   # Wheel radius in mm
        L1 = 119.35 # Half the distance between the centers of front and back wheels in mm
        L2 = 125.7  # Half the distance between the centers of left and right wheels in mm
        self.mecanum_xfer = (r/4) * np.array([[1,1,1,1],[-1,1,-1,1],\
                [1/(L1+L2),-1/(L1+L2),-1/(L1+L2),1/(L1+L2)]])

        # Initialize rangefinders
        self.rf1 = SimRangefinder(60.0, self.len_y/2.0, 90.0, self.dt)
        self.rf2 = SimRangefinder(-1 * self.len_x/2.0, -40.0, 180.0, self.dt)
        self.rf3 = SimRangefinder(0.0, -1 * self.len_y/2.0, 270.0, self.dt)
        self.rf4 = SimRangefinder(self.len_x/2.0, -40.0, 0.0, self.dt)
        self.imu = SimIMU(self.dt)

        # Initialize sim vars
        self.reset(False)

    def reset(self, randomize = True):
        # Reset vars
        self.time = 0
        self.reward = 0
        if (randomize):
            high = np.array([10, 0, 10, 0, np.pi, 1])
            self.state = self.state_start + np.random.uniform(low=-high, high=high)
        else:
            self.state = self.state_start
        self.terminal = 0
        self.init_state = self.state
        self.step([0,0,0,0,0])
        return self.obs

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    def step(self,u):
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
        x, xdot, y, ydot, th, thdot = self.state
        dt = self.dt

        u = np.clip(u, -2, 2)[0]
        self.time += dt
        self.action = np.array([u])

        # Desired trajectory inputs
        vd = 0 # ctrl[0]
        theta_d = 0 #np.pi * (ctrl[1] + 1)
        v_theta = 0 # ctrl[2]
    
        # Calculate voltage ratios for each motor to acheive desired trajectory
        v = np.zeros([4,1])
        v[2][0] = vd * np.sin(theta_d + np.pi/4) + v_theta
        v[3][0] = vd * np.cos(theta_d + np.pi/4) - v_theta
        v[1][0] = vd * np.cos(theta_d + np.pi/4) + v_theta
        v[0][0] = vd * np.sin(theta_d + np.pi/4) - v_theta

        # Normalize ratios to 1 if the maxval is > 1
        maxval = np.amax(np.absolute(v)) 
        v = v / maxval if (maxval > 1) else v

        # Convert voltage ratios to wheel rotational velocities in rad/sec
        # Model linear relationship between voltage and acceleration. Friction linear with velocity.
        # self.wheel_thetadotdot = v * self.wheel_thetadotdot - self.wheel_thetadot * self.friction_constant
        # self.wheel_thetadot = self.wheel_thetadot + self.wheel_thetadotdot * self.dt
        #wheel_w = self.max_wheel_w * v

        # Calculate robot frontwards, rightwards, and rotational velocities
        velocities = np.matmul(self.mecanum_xfer, self.wheel_thetadot)
        rightdot = velocities[0][0]
        frontdot = velocities[1][0]
        #thdot = velocities[2][0]

        # Calculate the x and y velocity components
        g = 10.
        m = 1.
        l = 1.
        newthdot = thdot + (-3*g/(2*l) * np.sin(th + np.pi) + 3./(m*l**2)*u) * dt
        newth = th + thdot*dt
        newthdot = np.clip(newthdot, -8, 8) #pylint: disable=E1111

        #self.theta = self.theta + (self.rotation_vel * TIME_STEP)
        
        newxdot = rightdot * np.cos(newth) + frontdot * np.sin(newth)
        newydot = rightdot * np.sin(newth) + frontdot * np.cos(newth)

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
        newx = np.clip(x + newxdot * dt, x_space, FIELD_XMAX - x_space)
        newy = np.clip(y + newydot * dt, y_space, FIELD_YMAX - y_space)

        self.state = np.array([newx, newxdot, newy, newydot, newth, newthdot])

        # Update sensor measurements
        self.updateSensors()
        
        # Get reward from executing the action
        self.updateReward()
        #cost = self.angle_normalize(self.theta)**2 + .1*self.thetadot**2 + .001*(ctrl[0]**2)
       
        # End of sim
        if (self.time > TIME_MAX):
            self.terminal = 1
            print(self.state, end = '       ;    ')
            print(self.obs)
        

        info = {}
        return self.obs, self.reward, self.terminal, info

    def render(self, mode='human'):

        if self.viewer is None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(500,500)
            self.viewer.set_bounds(-2.2,2.2,-2.2,2.2)
            rod = rendering.make_capsule(1, .2)
            rod.set_color(.8, .3, .3)
            self.pole_transform = rendering.Transform()
            rod.add_attr(self.pole_transform)
            self.viewer.add_geom(rod)
            axle = rendering.make_circle(.05)
            axle.set_color(0,0,0)
            self.viewer.add_geom(axle)
            fname = path.join(path.dirname(__file__), "images/robot.png")
            self.img = rendering.Image(fname, 1., 1.)
            self.imgtrans = rendering.Transform()
            self.img.add_attr(self.imgtrans)

        self.viewer.add_onetime(self.img)
        self.pole_transform.set_rotation(self.state[0] + np.pi/2)
        if self.last_u:
            self.imgtrans.scale = (-self.last_u/2, np.abs(self.last_u)/2)

        return self.viewer.render(return_rgb_array = mode=='rgb_array')

    def getReward(self):
        return self.reward


    def updateReward(self):      
        # Keep angle at 0
        self.reward_theta = -1.0 * self.angle_normalize(self.state[4])**2

        # Minimize effort
        self.reward_effort = -0.001 * self.action[0]**2
     
        # for action in self.action:
        #     self.reward_effort -= 0.001 * action**2

        # Rewarded for staying near center of field
        distance_from_center = pow(self.state[0] - FIELD_XMAX / 2,2) + pow(self.state[2] - FIELD_YMAX / 2,2)
        self.reward_dist =  -1.0 * (distance_from_center / 100.0)
        
        # Minimize velocities
        self.reward_vel = -0.01 * np.hypot(self.state[1], self.state[3])
        self.reward_rvel = -0.1 * self.state[5]**2

        # self.reward = self.reward_theta + self.reward_effort + self.reward_dist + self.reward_vel + self.reward_rvel
        self.reward = self.reward_theta + self.reward_effort + self.reward_rvel
        return self.reward

    def angle_normalize(self, x):
        return (((x+np.pi) % (2*np.pi)) - np.pi)

    def getState(self):
        return self.state

    def updateSensors(self):
        self.rf1.update(self.state)
        self.rf2.update(self.state)
        self.rf3.update(self.state)
        self.rf4.update(self.state)
        self.imu.update(self.state)
        self.obs = np.array([self.imu.cos, self.imu.sin, self.state[5]])
        return self.obs


class SimRangefinder():
    """ Simulates a rangefinder.
    x: x-position in robot coordinates
    y: y-position in robot coordinates
    theta: orientation in robot coordinates
    min_range: minimum measurement range
    max_range: maximum measurement range
    meas_period: amount of time between measurements
    """
    def __init__(self,x,y,theta,dt,max_range=1200.0,meas_period=0.033):
        self.dt = dt
        self.theta = np.radians(theta)
        self.position_theta = np.arctan2(y,x)
        self.radius = np.hypot(x,y)
        # rf_field_theta = robot.theta + self.position_theta
        # self.dim = [robot.x + self.radius * np.cos(rf_field_theta),\
        #     robot.y + self.radius * np.sin(rf_field_theta), 0.0, 0.0]
        self.max_range = max_range
        self.timebank = 0
        self.meas_period = meas_period
        self.meas = 0
        self.dim = [0,0,0,0]

    def update(self, state):
        """ Returns the measurement of the rangefinder in mm.
        robot: robot object
        """
        # Add some available time to the timebank
        self.timebank += self.dt
        # Only make a measurement if there's enough time in the bank
        if (self.timebank < self.meas_period):
            return
        # Use up some time from the timebank
        self.timebank -= self.meas_period

        # Calculate the sensor's position and orientation in the field
        # rf_field_theta is used to calculate where the sensor is but not where its oriented
        rf_field_theta = state[4] + self.position_theta
        x1 = state[0] + self.radius * np.cos(rf_field_theta)
        y1 = state[2] + self.radius * np.sin(rf_field_theta)

        # Calculate cos and sin of the angle of where the rangefinder is oriented
        rf_aim_theta = state[4]  + self.theta
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
            hy = meas_y / sinval if sinval != 0 else meas_y / 0.00000001 # If the hypotenuse extends to a horizontal wall

            # The correct measurement is the shorter one
            if (hx < hy):
                meas = hx
                x2 = x1 + meas_x
                y2 = y1 + meas * sinval
            else:
                meas = hy
                x2 = x1 + meas * cosval
                y2 = y1 + meas_y
        except:
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
        else:
            self.meas = -1


class SimIMU():
    def __init__(self,dt,meas_period = 0.001, sigma = 0.0):
        self.dt = dt
        self.timebank = 0
        self.cos = 0
        self.sin = 0
        self.meas_period = meas_period
        self.sigma = sigma

    def update(self, state):
        # Add some available time to the timebank
        self.timebank += self.dt
        # Only make a measurement if there's enough time in the bank
        if (self.timebank < self.meas_period):
            return
        # Use up some time from the timebank
        self.timebank -= self.meas_period

        # Add noise to measurement, convert to radians
        self.meas = state[4] #(robot.theta + np.random.normal(0, self.sigma)) % 360.0
        
        # Get x/y components of theta
        self.cos = np.cos(state[4])
        self.sin = np.sin(state[4])


class SimMicroSW():
    def __init__(self):
        return

