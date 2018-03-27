#!/usr/local/bin/python3
"""
Simulates the robot, playfield, sensors
Units are in millimeters, degrees, and seconds (for time).
"""

import sys
import csv
import arcade
import os
import numpy as np
import math
import helper_funcs as hf
import gym
import time
import robotsim as rs

SPRITE_SCALING = 1
BOX_WIDTH = 16

SCREEN_WIDTH = 1600
SCREEN_HEIGHT = 1000
MM_PER_PIX = 1.6

FRAME_SKIP = 0

class Simulator(arcade.Window):
    def __init__(self, width, height, cmdSeq, robot):
        super().__init__(width, height)

        # Set the working directory (where we expect to find files) to the same
        # directory this .py file is in. You can leave this out of your own
        # code, but it is needed to easily run the examples using "python -m"
        # as mentioned at the top of this program.
        file_path = os.path.dirname(os.path.abspath(__file__))
        os.chdir(file_path)

        # Set up the player
        self.score = 0
        self.player_sprite = None
        self.wall_list = None
        self.physics_engine = None

        # Setup robot and the command sequence to playback
        self.robot = robot
        self.step = -1
        self.max_step = len(cmdSeq)
        self.cmdSeq = cmdSeq;
        
    def setup(self):
        """ Set up the game and initialize the variables. """
        # Sprite lists
        self.all_sprites_list = arcade.SpriteList()
        self.wall_list = arcade.SpriteList()

        # Offset values in pixels to move (0,0) coordinate into field area
        self.xoffset = (SCREEN_WIDTH/2) - rs.FIELD_XMAX / (2 * MM_PER_PIX)
        self.yoffset = (SCREEN_HEIGHT/2) - rs.FIELD_YMAX / (2 * MM_PER_PIX)

        # Set up the player
        self.player_sprite = arcade.Sprite("images/robot.png", SPRITE_SCALING)
        self.all_sprites_list.append(self.player_sprite)

        # Create walls of field
        wall = arcade.Sprite("images/boxCrate_1556.png", SPRITE_SCALING)
        wall.center_x = int(SCREEN_WIDTH / 2)
        wall.center_y = int((SCREEN_HEIGHT + rs.FIELD_YMAX/MM_PER_PIX + BOX_WIDTH)/2)
        self.all_sprites_list.append(wall)
        self.wall_list.append(wall)

        wall = arcade.Sprite("images/boxCrate_1556.png", SPRITE_SCALING)
        wall.center_x = int(SCREEN_WIDTH / 2)
        wall.center_y = int((SCREEN_HEIGHT - rs.FIELD_YMAX/MM_PER_PIX - BOX_WIDTH)/2)
        self.all_sprites_list.append(wall)
        self.wall_list.append(wall)

        wall = arcade.Sprite("images/boxCrate_794.png", SPRITE_SCALING)
        wall.center_x = int((SCREEN_WIDTH + rs.FIELD_XMAX/MM_PER_PIX + BOX_WIDTH)/2)
        wall.center_y = int(SCREEN_HEIGHT / 2)
        self.all_sprites_list.append(wall)
        self.wall_list.append(wall)

        wall = arcade.Sprite("images/boxCrate_794.png", SPRITE_SCALING)
        wall.center_x = int((SCREEN_WIDTH - rs.FIELD_XMAX/MM_PER_PIX - BOX_WIDTH)/2)
        wall.center_y = int(SCREEN_HEIGHT / 2)
        self.all_sprites_list.append(wall)
        self.wall_list.append(wall)

        # Set the background color
        arcade.set_background_color(arcade.color.AMAZON)
        arcade.run()

    def on_draw(self):
        """
        Render the screen.
        """
        # This command has to happen before we start drawing
        arcade.start_render()

        # Draw all the sprites.
        self.wall_list.draw()
        self.player_sprite.draw()

        # Draw rangefinder sights
        self.draw_line_mm(self.robot.rf1.dim)
        self.draw_line_mm(self.robot.rf2.dim)
        self.draw_line_mm(self.robot.rf3.dim)
        self.draw_line_mm(self.robot.rf4.dim)

        #self.draw_motion_indicators()
        self.draw_motion_indicators2()
        self.draw_network_output_graph(700,51)

        # Print info text
        arcade.draw_text("Time: %fs" % (self.robot.time), 10, 70, arcade.color.BLACK, 12)
        arcade.draw_text("Reward: %6.1f Theta: %6.1f Effort: %6.1f Dist: %6.1f Vel: %6.1f  RVel: %6.1f" % 
            (self.robot.reward,self.robot.reward_theta, self.robot.reward_effort,
            self.robot.reward_dist,self.robot.reward_vel,self.robot.reward_rvel),
            10, 50, arcade.color.BLACK, 12)
        arcade.draw_text("Sensors: RF1: %8.2f, RF2: %8.2f, RF3: %8.2f, RF4: %8.2f, Mag: %3.1f" %\
                (self.robot.rf1.meas, self.robot.rf2.meas, self.robot.rf3.meas, self.robot.rf4.meas,\
                self.robot.imu.meas), 10, 30, arcade.color.BLACK, 12)
        arcade.draw_text("Robot: x: %8.2f, y: %8.2f, theta: %8.2f, thetadot: %8.2f)" % \
                (self.robot.state[0], self.robot.state[2], np.degrees(self.robot.state[4]), self.robot.state[5]), 10, 10, arcade.color.BLACK, 12)

    # Input array should be [x1,y1,x2,y2]
    def draw_line_mm(self,dim):
        arcade.draw_line(dim[0]/MM_PER_PIX + self.xoffset, dim[1]/MM_PER_PIX + self.yoffset, \
                dim[2]/MM_PER_PIX + self.xoffset, dim[3]/MM_PER_PIX + self.yoffset,
                arcade.color.WHITE, 4)

    def draw_motion_indicators(self):
        for i in range(0,4):
            # Calculate location of motor
            motor_field_theta = self.robot.state[4] + self.robot.motor_angles[i]
            x = self.robot.state[0] + self.robot.motor_radius * math.cos(motor_field_theta)
            y = self.robot.state[2] + self.robot.motor_radius * math.sin(motor_field_theta)
            x = x/MM_PER_PIX + self.xoffset
            y = y/MM_PER_PIX + self.yoffset

            # Draw arc around motor
            self.draw_arrow_arc(x,y,self.robot.action[i])

    def draw_motion_indicators2(self):
        x = self.robot.state[0]/MM_PER_PIX + self.xoffset
        y = self.robot.state[2]/MM_PER_PIX + self.yoffset
        self.draw_arrow_arc(x,y,-self.robot.action)
        #self.draw_radial_arrow(x,y,self.robot.action[0],-1*(self.robot.action[1]+1)*np.pi+self.robot.theta)

    def draw_network_output_graph(self,x,y):
        width = 60
        spacer = 5
        num_drawn = 0
        max_height = 45
        for idx, action in enumerate(self.robot.action):
            try:
                x_tmp = x + idx * width
                height = max_height * action / 2
                arcade.draw_rectangle_filled(x_tmp, y + height / 2, width - spacer, height, arcade.color.WHITE)
                arcade.draw_text("%d" % (idx+1), x_tmp, y + 2, arcade.color.BLACK, 12, align='center', anchor_x='center')
                arcade.draw_text("%+0.2f" % (action), x_tmp, y - 14, arcade.color.BLACK, 12, align='center', anchor_x='center')
                num_drawn += 1
            except:
                pass
        arcade.draw_rectangle_outline(x + (num_drawn - 1) * width / 2, y, width * num_drawn, max_height * 2 + 2, arcade.color.BLACK)
        arcade.draw_line(x - width/2, y, x + num_drawn * width - width/2, y, arcade.color.BLACK, 2)

    def draw_radial_arrow(self,x,y,mag,theta):
        x2 = x + 100 * mag * np.cos(theta)
        y2 = y + 100 * mag * np.sin(theta)
        arcade.draw_line(x, y, x2, y2, arcade.color.WHITE, 5)
        self.draw_center_triangle_filled(x2,y2,10 if mag > 0 else -10,theta)

    # Draws an equilateral triangle centered at x,y with a vertex at angle theta
    def draw_center_triangle_filled(self,x,y,mag,theta):
        x1 = x + mag * np.cos(theta)
        y1 = y + mag * np.sin(theta)
        x2 = x + mag * np.cos(theta + 2*np.pi/3)
        y2 = y + mag * np.sin(theta + 2*np.pi/3)
        x3 = x + mag * np.cos(theta + 4*np.pi/3)
        y3 = y + mag * np.sin(theta + 4*np.pi/3)
        arcade.draw_triangle_filled(x1, y1, x2, y2, x3, y3, arcade.color.WHITE)

    # Draws a rotational arrow centered at (x,y), direction and scale set by magnitude
    def draw_arrow_arc(self,x,y,mag):
        width = abs(mag * 90)
        tri_len = 7
        tri_x = x + width
        if (mag > 0):
            angle = 0
            tri_y = y - tri_len * 2
        else:
            angle = 90
            tri_y = y + tri_len * 2

        arcade.draw_arc_outline(x,y,width,width,arcade.color.WHITE,0,270,5,angle)
        arcade.draw_triangle_filled(tri_x + tri_len, y, \
                tri_x - tri_len, y, \
                tri_x, tri_y, arcade.color.WHITE)

#    def on_key_press(self, key, modifiers):
#        """Called whenever a key is pressed. """
#        if key == arcade.key.UP:
#            self.player_sprite.change_y = MOVEMENT_SPEED
#        elif key == arcade.key.DOWN:
#            self.player_sprite.change_y = -MOVEMENT_SPEED
#        elif key == arcade.key.LEFT:
#            self.player_sprite.change_x = -MOVEMENT_SPEED
#        elif key == arcade.key.RIGHT:
#            self.player_sprite.change_x = MOVEMENT_SPEED
#
#    def on_key_release(self, key, modifiers):
#        """Called when the user releases a key. """
#        if key == arcade.key.UP or key == arcade.key.DOWN:
#            self.player_sprite.change_y = 0
#        elif key == arcade.key.LEFT or key == arcade.key.RIGHT:
#            self.player_sprite.change_x = 0

    def update(self, delta_time):
        """ Movement and game logic """
        # Track frame number to allow skipping
        self.frame = -1
        while (self.frame < FRAME_SKIP):
            self.frame += 1

            # Next step of robot simulation
            self.step += 1
            if (self.step == self.max_step):
                quit()
            self.robot.step(self.cmdSeq[self.step])
            time.sleep(0.01)

        # Update graphics vars
        self.player_sprite.position[0] = self.robot.state[0]/MM_PER_PIX + self.xoffset
        self.player_sprite.position[1] = self.robot.state[2]/MM_PER_PIX + self.yoffset
        self.player_sprite.angle = np.degrees(self.robot.state[4])


def main():
    #quit()
    try:
        #Parse the action log
        action_log_file = sys.argv[1]
        with open(action_log_file, 'r') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_NONNUMERIC)
            action_log = []
            first = True
            for row in spamreader:
                if (first):
                    first = False
                    state_start = row
                else:
                    action_log.append(row)
    except:
        # Make cmds
        max_step = 1000
        state_start = np.array([rs.FIELD_XMAX / 2, 0, rs.FIELD_YMAX / 2, 0, 0, 0])

        #action_log = np.zeros([max_step,4])
        action_log = np.zeros([max_step,1])
        for i in range(0,max_step):
            action_log[i] = [0.1] #[0.1, 0, 0.0, 0]

    robot = rs.SimRobot(state_start)
    game = Simulator(SCREEN_WIDTH, SCREEN_HEIGHT, action_log, robot)
    game.setup()

if __name__ == "__main__":
    main()
