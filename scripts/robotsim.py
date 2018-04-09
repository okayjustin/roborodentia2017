#!/usr/local/bin/python3
"""
Simulates the robot, playfield, sensors
Units are in millimeters, degrees, and seconds (for time).
"""

from sensors import *
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
        self.sensors = []                                                      # Sensor index
        self.sensors.append(SimIMU(self.dt))                                        # 0
        self.sensors.append(SimRangefinder(60.0, LEN_Y/2.0, 90.0, self.dt, FIELD_XMAX, FIELD_YMAX))         # 1
        self.sensors.append(SimRangefinder(-1 * LEN_X/2.0, -40.0, 180.0, self.dt, FIELD_XMAX, FIELD_YMAX))  # 2
        self.sensors.append(SimRangefinder(0.0, -1 * LEN_Y/2.0, 270.0, self.dt, FIELD_XMAX, FIELD_YMAX))    # 3
        self.sensors.append(SimRangefinder(LEN_X/2.0, -40.0, 0.0, self.dt, FIELD_XMAX, FIELD_YMAX))         # 4
        self.sensors.append(SimDesiredXY('x', FIELD_XMAX, FIELD_YMAX, LEN_X, LEN_Y))                            # 5
        self.sensors.append(SimDesiredXY('y', FIELD_XMAX, FIELD_YMAX, LEN_X, LEN_Y))                            # 6
        self.sensors.append(SimActualXY('x', FIELD_XMAX, FIELD_YMAX, LEN_X, LEN_Y))                             # 7
        self.sensors.append(SimActualXY('y', FIELD_XMAX, FIELD_YMAX, LEN_X, LEN_Y))                             # 8

        # Sensor indices to include in observation for each network
        self.obs_settings = [[0],       # Angle network
                             [7],    # Trans X network
                             [6, 8]]    # Trans Y network

        # Add sensors depending on what network is being trained
        if (train == 'angle'):
            act_dim = 1
            self.net_index = 0

        elif (train == 'transx' or train == 'transy'):
            act_dim = 1

            if (train == 'transx'):
                self.net_index = 1
            else:
                self.net_index = 2

            # Load angle control ann
            print("Loading angle ANN")
            angle_model_path = './results/models-angle/model.ckpt'
            self.angle_ann = ann(angle_model_path, state_dim = 3, action_dim = 1, action_space_high = 2.0)

        elif (train == 'pathfind'):
            self.net_index = 3
            pass

        # Calculate the number of elements in the observation array
        obs_high = np.array([])
        obs_low = np.array([])
        state_dim = 0
        for sensor_index in self.obs_settings[self.net_index]:
            sensor = self.sensors[sensor_index]
            obs_high = np.append(obs_high, sensor.high)
            obs_low = np.append(obs_low, sensor.low)
            state_dim += len(sensor.meas)
        self.observation_space = gym.spaces.box.Box(low=obs_low, high=obs_high)

        act_high = 2.0
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
        self.updateObservation()

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
        # u[0]: Desired rotational speed, -1 to 1, +1 CW, -1 CCW
        # u[1]: Desired field x velocity
        # u[2]: Desired field y velocity
        # u[3]: Desired field x position
        # u[4]: Desired field y position
        if self.train == 'angle':
            u = np.append(u, np.zeros(self.u_len - 1))
        elif self.train == 'transx':
            # Get input from angle control ANN
            u_angle = self.angle_ann.predict(self.obs_sets[0])
            u = np.append(u_angle, u)

            # Emulate a control input that keeps the current desired location
            u_transy = 0
            u_desx = np.interp(x_des, [LEN_X/2, FIELD_XMAX - LEN_X/2], [-2,2])
            u_desy = np.interp(y_des, [LEN_Y/2, FIELD_YMAX - LEN_Y/2], [-2,2])
            u = np.append(u, [u_transy, u_desx, u_desy])
        elif self.train == 'transy':
            # Get input from angle control ANN
            u_angle = self.angle_ann.predict(self.obs_sets[0])

            # Get input from x translation control ANN
            #u_transx = self.transx_ann.predict(self.obs_sets[1])

            u = np.append(u_angle, u_transx)
            u = np.append(u_angle, u)

            # Emulate a control input that keeps the current desired location
            u_desx = np.interp(x_des, [LEN_X/2, FIELD_XMAX - LEN_X/2], [-2,2])
            u_desy = np.interp(y_des, [LEN_Y/2, FIELD_YMAX - LEN_Y/2], [-2,2])
            u = np.append(u, [u_desx, u_desy])
        elif self.train == 'pathfind':
            # Determine new desired location
            u = np.zeros(self.u_len)

        u = np.clip(u, -2, 2)
        #print(u)
        self.last_u = u

        #  Trajectory control inputs
        v_theta = u[0]
        vd = np.hypot(u[1],u[2]) / 2.0
        #print("%f | %f" % (u[1],vd))
        theta_d = np.arctan2(u[2],u[1]) - self.sensors[0].prevth # Adjust desired translation angle to field coordinates

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
        self.updateObservation()

        # Get reward from executing the action
        self.updateReward()

        # End of sim
        if (self.time > TIME_MAX):
            self.terminal = 1
        info = {}
        return self.obs, self.reward, self.terminal, info


    def updateObservation(self):
        # Holds observations for each network
        obs_sets = []

         # Update sensor measurements
        for sensor in self.sensors:
            sensor.update(self.state)

        # Calculate the various observation sets
        for sensor_indices in self.obs_settings:
            obs = np.array([])
            for sensor_index in sensor_indices:
                sensor = self.sensors[sensor_index]
                obs = np.append(obs, sensor.meas)
            obs_sets.append(obs)

        self.obs_sets = obs_sets
        self.obs = obs_sets[self.net_index]
        #print(self.obs)

    def updateReward(self):
        if (self.train == 'angle'):
            # Keep angle at 0
            self.reward_theta = -1.0 * self.angle_normalize(self.state[4])**2
            # Minimize effort
            self.reward_effort = -0.001 * self.last_u[0]**2
            # Minimize velocities
            self.reward_rvel = -0.1 * self.state[5]**2
            self.reward = self.reward_theta + self.reward_rvel


        elif (self.train == 'transx'):
            # Rewarded for staying near desired x coordinate
            self.reward_dist = -0.00001 * pow(self.state[0] - self.state[6], 2)
            # Minimize velocities
            self.reward_vel = -0.000003 * self.state[1]**2
            #print("Rewards: %f, %f" % (self.reward_dist, self.reward_vel))
            self.reward = self.reward_dist + self.reward_vel


        elif (self.train == 'transy'):
            # Rewarded for staying near desired y coordinate
            self.reward_dist = -0.00001 * pow(self.state[2] - self.state[7], 2)
            # Minimize velocities
            self.reward_vel = -0.000001 * self.state[3]**2
            self.reward = self.reward_dist + self.reward_vel
        #print(self.reward)
        return self.reward

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

