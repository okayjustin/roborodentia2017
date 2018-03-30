#!/usr/local/bin/python3
"""
Simulates the robot, playfield, sensors
Units are in millimeters, degrees, and seconds (for time).
"""

import numpy as np
import helper_funcs as hf
import gym
from gym.utils import seeding
from os import path
import pyglet
from pyglet.gl import *

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
        self.viewer = None

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
        self.dt = 0.01
        self.time = 0
        self.reward = 0
        self.terminal = 0
        self.obs = [0,0,0]

        # Mecanum equation converts wheel velocities to tranlational x,y, and rotational velocities
        self.wheel_max_thetadotdot = 430                                # Max wheel rotational acceleration in rad/s^2
        self.wheel_max_thetadot = 24.46                                 # Max wheel rotational velocity in rad/s
        self.friction_constant = self.wheel_max_thetadotdot/self.wheel_max_thetadot # Friction constant in 1/s
        self.wheel_thetadotdot =  np.zeros([4,1])                       # Wheel rotational acceleration in rad/s/2
        self.wheel_thetadot = np.zeros([4,1])                           # Wheel rotational velocities in rad/s
        r =  30.0   # Wheel radius in mm
        L1 = 119.35 # Half the distance between the centers of front and back wheels in mm
        L2 = 125.7  # Half the distance between the centers of left and right wheels in mm
        self.mecanum_xfer = (r/4) * np.array([[1,1,1,1],[-1,1,-1,1],\
                [1/(L1+L2),-1/(L1+L2),-1/(L1+L2),1/(L1+L2)]])

        # Initialize rangefinders
        self.sensors = []
#        self.sensors.append(SimRangefinder(60.0, self.len_y/2.0, 90.0, self.dt))
#        self.sensors.append(SimRangefinder(-1 * self.len_x/2.0, -40.0, 180.0, self.dt))
#        self.sensors.append(SimRangefinder(0.0, -1 * self.len_y/2.0, 270.0, self.dt))
#        self.sensors.append(SimRangefinder(self.len_x/2.0, -40.0, 0.0, self.dt))
        self.sensors.append(SimIMU(self.dt))

        # Set up interface with nn
        high = np.array([])
        low = np.array([])
        for sensor in self.sensors:
            high = np.append(high, sensor.high)
            low = np.append(low, sensor.low)
        self.action_space = gym.spaces.box.Box(low=-2.0, high=2.0, shape=(1,))
        self.observation_space = gym.spaces.box.Box(low=low, high=high)

        # Initialize sim vars
        self.reset(False)

    def reset(self, randomize = True):
        # Reset vars
        self.time = 0
        self.reward = 0
        if (randomize):
            high = np.array([100, 0, 100, 0, 0.2, 0])
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
        # Calculate motor voltages for the specified control inputs
        # ctrl[0]: Desired rotational speed, -1 to 1, +1 CW, -1 CCW
        # ctrl[1]: Desired robot speed, -1 to 1
        # ctrl[2]: Desired translation angle, -1 to 1, +1 dead right, 0.5 dead up, 0 dead left, -0.5 dead down
        x, xdot, y, ydot, th, thdot = self.state
        dt = self.dt

        u = np.clip(u, -2, 2)
        self.last_u = u # for rendering
        self.time += dt

        # Desired trajectory inputs
        v_theta = u[0] # ctrl[2]
        vd = 0#u[1]
        theta_d = 0#u[2] * np.pi - self.sensors[4].prevth

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
        self.wheel_thetadotdot = v * self.wheel_max_thetadotdot - self.wheel_thetadot * self.friction_constant
        self.wheel_thetadot = self.wheel_thetadot + self.wheel_thetadotdot * self.dt

        # Calculate robot frontwards, rightwards, and rotational velocities
        velocities = np.matmul(self.mecanum_xfer, self.wheel_thetadot)
        rightdot = velocities[0][0]
        frontdot = velocities[1][0]
        newthdot = velocities[2][0]

        # Calculate the new theta
        newth = th + newthdot*dt

        # Calcuate x and y velocity components
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
        obs = np.array([])
        for sensor in self.sensors:
            sensor.update(self.state)
            obs = np.append(obs, sensor.meas)
        self.obs = obs

        # Get reward from executing the action
        self.updateReward()

        # End of sim
        if (self.time > TIME_MAX):
            self.terminal = 1

        info = {}
        return self.obs, self.reward, self.terminal, info

    def render(self, mode='human'):
        if self.viewer is None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(1300,650)
            self.viewer.set_bounds(0,FIELD_XMAX,0,FIELD_YMAX)

            # Robot image
            fname = path.join(path.dirname(__file__), "images/robot.png")
            self.img_robot = rendering.Image(fname, self.len_x, self.len_y)
            self.img_robot.set_color(1,1,1)
            self.imgtrans_robot = rendering.Transform()
            self.img_robot.add_attr(self.imgtrans_robot)
            self.viewer.add_geom(self.img_robot)

            # Rotation direction indicator
            fname = path.join(path.dirname(__file__), "images/clockwise.png")
            self.img_rot_ind = rendering.Image(fname, self.len_x, self.len_x)
            #self.img_robot.set_color(0,0,0)
            self.imgtrans_rot_ind = rendering.Transform()
            self.img_rot_ind.add_attr(self.imgtrans_rot_ind)
            self.viewer.add_geom(self.img_rot_ind)

        # Robot
        self.imgtrans_robot.set_translation(self.state[0], self.state[2])
        self.imgtrans_robot.set_rotation(self.state[4])

        # Motion indicators
        self.imgtrans_rot_ind.scale = (self.last_u[0]/2, np.abs(self.last_u[0])/2)
        self.imgtrans_rot_ind.set_translation(self.state[0], self.state[2])
#        self.draw_radial_arrow(self.state[0], self.state[2], self.last_u[1],-1*(self.last_u[2]+1)*np.pi+self.th)

        # Rangefinder lines
        for sensor in self.sensors:
            if (sensor.type == 'Rangefinder'):
                self.viewer.draw_line(self.sensors[i].dim[0], self.sensors[i].dim[1])

        return self.viewer.render(return_rgb_array = mode=='rgb_array')

    def draw_radial_arrow(self,x,y,mag,theta):
        x2 = x + 100 * mag * np.cos(theta)
        y2 = y + 100 * mag * np.sin(theta)
        self.viewer.draw_line((x, y), (x2, y2))
        self.draw_center_triangle_filled(x2,y2,10 if mag > 0 else -10,theta)

    # Draws an equilateral triangle centered at x,y with a vertex at angle theta
    def draw_center_triangle_filled(self,x,y,mag,theta):
        x1 = x + mag * np.cos(theta)
        y1 = y + mag * np.sin(theta)
        x2 = x + mag * np.cos(theta + 2*np.pi/3)
        y2 = y + mag * np.sin(theta + 2*np.pi/3)
        x3 = x + mag * np.cos(theta + 4*np.pi/3)
        y3 = y + mag * np.sin(theta + 4*np.pi/3)
        self.viewer.draw_polygon(((x1, y1), (x2, y2), (x3, y3)))

    def updateReward(self):
        # Keep angle at 0
        self.reward_theta = -1.0 * self.angle_normalize(self.state[4])**2

        # Minimize effort
        self.reward_effort = -0.001 * self.last_u[0]**2

        # for u in self.last_u:
        #     self.reward_effort -= 0.001 * action**2

        # Rewarded for staying near center of field
        distance_from_center = pow(self.state[0] - FIELD_XMAX / 2,2) + pow(self.state[2] - FIELD_YMAX / 2,2)
        self.reward_dist =  -1.0 * (distance_from_center / 100.0)

        # Minimize velocities
        self.reward_vel = -0.01 * np.hypot(self.state[1], self.state[3])
        self.reward_rvel = -0.1 * self.state[5]**2

        # self.reward = self.reward_theta + self.reward_effort + self.reward_dist + self.reward_vel + self.reward_rvel
        self.reward = self.reward_theta + self.reward_rvel# + self.reward_effort
        return self.reward

    def angle_normalize(self, x):
        return (((x+np.pi) % (2*np.pi)) - np.pi)



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
        self.type = 'Rangefinder'
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
        self.meas = np.array([0.,0.])
        self.dim = [(0.,0.),(0.,0.)]
        self.high = np.array([1200., 750.])
        self.low = np.array([-1., -750.])

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
            dist_x = FIELD_XMAX - x1
        else:
            dist_x = -1 * x1
        if sinval >= 0:
            dist_y = FIELD_YMAX - y1
        else:
            dist_y = -1 * y1

        # Calculate the two possible measurements
        try:
            hx = dist_x / cosval # If the hypotenuse extends to a vertical wall
            hy = dist_y / sinval if sinval != 0 else dist_y / 0.00000001 # If the hypotenuse extends to a horizontal wall

            # The correct measurement is the shorter one
            if (hx < hy):
                dist = hx
                x2 = x1 + dist_x
                y2 = y1 + dist * sinval
            else:
                dist = hy
                x2 = x1 + dist * cosval
                y2 = y1 + dist_y
        except:
            if (cosval != 0):
                dist = dist_x / cosval
                x2 = x1 + dist_x
                y2 = y1 + dist * sinval
            else:
                dist = dist_y / sinval
                x2 = x1 + dist * cosval
                y2 = y1 + dist_y

        # Add noise to measurement
        #sigma = 2E-05 * pow(dist,2) - 0.0062 * dist + 2.435
        #dist += np.random.normal(0, sigma)

        # Only update if measurement is in range
        if ((dist > 30) and (dist < 1200)):
            self.dim = [(x1,y1),(x2,y2)]
            dist_dot = (dist - self.meas[0]) / self.dt
            self.meas = np.array([dist, dist_dot])
        else:
            self.meas = np.array([-1.0, -1.0])


class SimIMU():
    def __init__(self,dt,meas_period = 0.001, sigma = 0.0):
        self.type = 'IMU'
        self.dt = dt
        self.timebank = 0
        self.prevth = 0
        self.meas = np.array([0., 0., 0.])
        self.meas_period = meas_period
        self.sigma = sigma
        self.high = np.array([1., 1., 8.])
        self.low = np.array([-1., -1., -8.])

    def update(self, state):
        # Add some available time to the timebank
        self.timebank += self.dt
        # Only make a measurement if there's enough time in the bank
        if (self.timebank < self.meas_period):
            return
        # Use up some time from the timebank
        self.timebank -= self.meas_period

        # Get angle measurement and estimate a velocity
             #(robot.theta + np.random.normal(0, self.sigma)) % 360.0
        th = state[4]
        thdot = (th - self.prevth) / self.dt
        self.prevth = th

        # Get x/y components of theta
        cos = np.cos(th)
        sin = np.sin(th)

        self.meas = np.array([cos, sin, thdot])

class SimMicroSW():
    def __init__(self):
        return
