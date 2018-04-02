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
from pi_client import ann

FIELD_XMAX = 2438.4 # Maximum x dimension in mm
FIELD_YMAX = 1219.2  # Maximum y dimension in mm
MOTOR_DELTA = 10 # Controls percentage of how much motors differ . Ex. 5% means parameter will vary up to 5%
TIME_MAX = 10 #seconds

# Robot dimensions
LEN_X = 315.0      # length of the robot left to right
LEN_Y = 275.0      # length of the robot front to back

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
    state_start: starting state of robot with 8 elements
        0: x position
        1: x velocity
        2: y position
        3: y velocity
        4: rotational position theta
        5: rotational velocity
        6: desired x position
        7: desired y position
        8: balls present in front left hopper (boolean)
        9: balls present in back left hopper (boolean)
        10: balls present in back right hopper (boolean)
        11: balls present in front right hopper (boolean)
    """
    def __init__(self, train = None,
            state_start = [FIELD_XMAX/2, 0, FIELD_YMAX/2, 0, 0, 0, FIELD_XMAX/2, FIELD_YMAX/2, 0, 0, 0, 0]):
        # For rendering
        self.viewer = None
        self.dt = 0.01
        self.train = train
        self.state_start = np.array(state_start)

        # Length of control input u
        self.u_len = 5

        # Magnitude and angle of a line between the center and top right corner
        self.diag_angle = np.arctan(LEN_Y / LEN_X)
        self.diag_len = np.hypot(LEN_X,LEN_Y) / 2

        # Mecanum equation converts wheel velocities to tranlational x,y, and rotational velocities
        self.wheel_max_thetadotdot = 430                                # Max wheel rotational acceleration in rad/s^2
        self.wheel_max_thetadot = 24.46                                 # Max wheel rotational velocity in rad/s
        self.friction_constant = self.wheel_max_thetadotdot/self.wheel_max_thetadot # Friction constant in 1/s
        self.wheel_thetadotdot =  np.zeros(4)                       # Wheel rotational acceleration in rad/s/2
        self.wheel_thetadot = np.zeros(4)                           # Wheel rotational velocities in rad/s
        r =  30.0   # Wheel radius in mm
        L1 = 119.35 # Half the distance between the centers of front and back wheels in mm
        L2 = 125.7  # Half the distance between the centers of left and right wheels in mm
        self.mecanum_xfer = (r/4) * np.array([[1,1,1,1],[-1,1,-1,1],\
                [1/(L1+L2),-1/(L1+L2),-1/(L1+L2),1/(L1+L2)]])

        # Set up action space and observation space
        self.sensors = []
        obs_high = np.array([])
        obs_low = np.array([])
        act_high = 2.0

        # Add sensors depending on what network is being trained
        if (train == 'angle'):
            act_dim = 1
            self.sensors.append(SimIMU(self.dt))
        elif (train == 'translation'):
            act_dim = 2
            self.sensors.append(SimIMU(self.dt))
            self.sensors.append(SimRangefinder(60.0, LEN_Y/2.0, 90.0, self.dt))
            self.sensors.append(SimRangefinder(-1 * LEN_X/2.0, -40.0, 180.0, self.dt))
            self.sensors.append(SimRangefinder(0.0, -1 * LEN_Y/2.0, 270.0, self.dt))
            self.sensors.append(SimRangefinder(LEN_X/2.0, -40.0, 0.0, self.dt))
            self.sensors.append(SimDesiredXY())

            # Load angle control ann
            print("Loading angle ANN")
            angle_model_path = './results/models-angle/model.ckpt'
            self.angle_ann = ann(angle_model_path, state_dim = 3, action_dim = 1, action_space_high = act_high)
        elif (train == 'pathfind'):
            pass

        # Calculate the number of elements in the observation array
        state_dim = 0
        for sensor in self.sensors:
            obs_high = np.append(obs_high, sensor.high)
            obs_low = np.append(obs_low, sensor.low)
            state_dim += len(sensor.meas)
        self.observation_space = gym.spaces.box.Box(low=obs_low, high=obs_high)
        self.action_space = gym.spaces.box.Box(low=-act_high, high=act_high, shape=(act_dim,))

        # Initialize sim vars
        self.reset(False)

    def reset(self, randomize = True):
        # Reset vars
        self.time = 0
        self.reward = 0
        self.terminal = 0
        self.last_u = np.zeros(self.u_len)

        if (randomize):
            high = np.array([FIELD_XMAX/2 - LEN_X/2, 0,
                             FIELD_YMAX/ - LEN_Y/2, 0,
                             0.2, 0,
                             FIELD_XMAX/2 - LEN_X/2, FIELD_YMAX/2 - LEN_Y/2,
                             0, 0, 0, 0])
            self.state = self.state_start + np.random.uniform(low=-high, high=high)

            # Randomize variable friction and voltage sensitivity per wheel
            self.friction_var = np.array([self.friction_constant * (1+np.random.uniform(-MOTOR_DELTA, MOTOR_DELTA) / 100) for i in range(4)])
            self.v_sensitivity = np.array([1 + np.random.uniform(-MOTOR_DELTA, MOTOR_DELTA)/100 for i in range(4)])
        else:
            self.state = self.state_start
            self.friction_var = np.array([self.friction_constant for i in range(4)])
            self.v_sensitivity = np.array([1 for i in range(4)])

        # Update sensors and observation array
        obs = []
        for sensor in self.sensors:
            sensor.update(self.state)
            obs = np.append(obs, sensor.meas)
        self.obs = np.array(obs)

        return self.obs

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self,u):
        # Get state vars
        x, xdot, y, ydot, th, thdot, x_des, y_des, hopfl, hopbl, hopbr, hopfr = self.state
        dt = self.dt
        self.time += dt

        # Determine the control input array u depending on what's being trained
        # u[0]: jesired rotational speed, -1 to 1, +1 CW, -1 CCW
        # u[1]: Desired robot speed, -1 to 1
        # u[2]: Desired translation angle, -1 to 1, +1 dead right, 0.5 dead up, 0 dead left, -0.5 dead down
        # u[3]: Desired x position
        # u[4]: Desired y position
        if self.train == 'angle':
            u = np.append(u, np.zeros(self.u_len - 1))
        elif self.train == 'translation':
            # Get input from angle control ANN
            obs_angle = self.obs[0:3]
            u_angle = self.angle_ann.predict(obs_angle)

            u = np.append(u_angle, u)

            # Emulate a control input that keeps the current desired location
            u_desx = np.interp(x_des, [LEN_X/2, FIELD_XMAX - LEN_X/2], [-2,2])
            u_desy = np.interp(y_des, [LEN_Y/2, FIELD_YMAX - LEN_Y/2], [-2,2])
            u = np.append(u, [u_desx, u_desy])

        elif self.train == 'pathfind':
            # Determine new desired location
            u = np.zeros(self.u_len)

        u = np.clip(u, -2, 2)
        self.last_u = u

        #  Trajectory control inputs
        v_theta = u[0]
        vd = u[1]
        theta_d = u[2] * np.pi / 2 - self.sensors[0].prevth # Adjust desired translation angle to field coordinates

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
        self.wheel_thetadotdot = np.multiply(self.v_sensitivity, v) * self.wheel_max_thetadotdot - np.multiply(self.wheel_thetadot, self.friction_var)
        self.wheel_thetadot = self.wheel_thetadot + self.wheel_thetadotdot * self.dt

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

        # Determine new desired location
        newxdes = np.interp(u[3], [-2,2], [LEN_X/2, FIELD_XMAX - LEN_X/2])
        newydes = np.interp(u[4], [-2,2], [LEN_Y/2, FIELD_YMAX - LEN_Y/2])

        # Determine hopper states
        newhopfl = False
        newhopbl = False
        newhopbr = False
        newhopfr = False

        # Determine new state
        self.state = np.array([newx, newxdot, newy, newydot, newth, newthdot, newxdes, newydes, newhopfl, newhopbl, newhopbr, newhopfl])

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
        self.imgtrans_rot_ind.scale = (self.last_u[0]/2, np.abs(self.last_u[0])/2)
        self.imgtrans_rot_ind.set_translation(self.state[0], self.state[2])
#        self.draw_radial_arrow(self.state[0], self.state[2], self.last_u[1],-1*(self.last_u[2]+1)*np.pi+self.th)

        # Rangefinder lines
        for sensor in self.sensors:
            if (sensor.type == 'Rangefinder'):
                self.viewer.draw_line(sensor.dim[0], sensor.dim[1])

        # Desired location indicator
        self.viewer.draw_line([self.state[0], self.state[2]], [self.state[6], self.state[7]])

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
        if (self.train == 'angle'):
            # Keep angle at 0
            self.reward_theta = -1.0 * self.angle_normalize(self.state[4])**2
            # Minimize effort
            self.reward_effort = -0.001 * self.last_u[0]**2
            # Minimize velocities
            self.reward_rvel = -0.1 * self.state[5]**2

            self.reward = self.reward_theta + self.reward_rvel

        elif (self.train == 'translation'):
            # Rewarded for staying near desired x,y coordinate
            dist_from_desired = pow(self.state[0] - self.state[6], 2) + pow(self.state[2] - self.state[7], 2)
            self.reward_dist =  -0.001 * dist_from_desired
            # Minimize velocities
            self.reward_vel = -0.1 * np.hypot(self.state[1], self.state[3])

            self.reward = self.reward_dist + self.reward_vel

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

class SimDesiredXY():
    def __init__(self):
        self.type = 'DesiredXY'
        self.meas = np.array([0., 0.])
        self.high = np.array([FIELD_XMAX - LEN_X/2, FIELD_YMAX - LEN_Y/2])
        self.low = np.array([LEN_X/2, LEN_Y/2])

    def update(self, state):
        self.meas = np.array([state[6], state[7]])

class SimMicroSW():
    def __init__(self):
        return
