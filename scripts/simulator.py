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
import robotsim as rs

SPRITE_SCALING = 1
BOX_WIDTH = 16

SCREEN_WIDTH = 1600
SCREEN_HEIGHT = 1000
MM_PER_PIX = 1.6

FRAME_SKIP = 0

class Simulator(arcade.Window):
    def __init__(self, width, height, cmdSeq):
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

        # Setup robot and the command sequence to playbac
        self.robot = rs.SimRobot()
        self.step = -1
        self.max_step = len(cmdSeq)
        self.cmdSeq = cmdSeq;
        self.setup()

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

        self.draw_motion_indicators()

        # Print info text
        arcade.draw_text("Time: %fs" % (self.robot.time), 10, 70, arcade.color.BLACK, 12)
        arcade.draw_text("Reward: %f" % (self.robot.reward), 10, 50, arcade.color.BLACK, 12)
        arcade.draw_text("Sensors: RF1: %8.2f, RF2: %8.2f, RF3: %8.2f, RF4: %8.2f, Mag: %3.1f" %\
                (self.robot.rf1.meas, self.robot.rf2.meas, self.robot.rf3.meas, self.robot.rf4.meas,\
                self.robot.imu.meas), 10, 30, arcade.color.BLACK, 12)
        arcade.draw_text("Robot: x: %8.2f, y: %8.2f, theta: %8.2f)" % \
                (self.robot.x, self.robot.y, self.robot.theta), 10, 10, arcade.color.BLACK, 12)

    # Input array should be [x1,y1,x2,y2]
    def draw_line_mm(self,dim):
        arcade.draw_line(dim[0]/MM_PER_PIX + self.xoffset, dim[1]/MM_PER_PIX + self.yoffset, \
                dim[2]/MM_PER_PIX + self.xoffset, dim[3]/MM_PER_PIX + self.yoffset,
                arcade.color.WOOD_BROWN, 3)

    def draw_motion_indicators(self):
        for i in range(0,4):
            # Calculate location of motor
            motor_field_theta = math.radians(self.robot.theta) + self.robot.motor_angles[i]
            x = self.robot.x + self.robot.motor_radius * math.cos(motor_field_theta)
            y = self.robot.y + self.robot.motor_radius * math.sin(motor_field_theta)
            x = x/MM_PER_PIX + self.xoffset
            y = y/MM_PER_PIX + self.yoffset

            # Draw arc around motor
            self.draw_arrow_arc(x,y,self.robot.action[i])


    # Draws a rotational arrow centered at (x,y), direction and scale set by magnitude
    def draw_arrow_arc(self,x,y,mag):
        tri_len = 7
        width = abs(mag * 30)
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

        # Update graphics vars
        self.player_sprite.position[0] = self.robot.x/MM_PER_PIX + self.xoffset
        self.player_sprite.position[1] = self.robot.y/MM_PER_PIX + self.yoffset
        self.player_sprite.angle = self.robot.theta


def main():
    # Parse the action log
    action_log_file = sys.argv[1]
    with open(action_log_file, 'r') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_NONNUMERIC)
        action_log = []
        for row in spamreader:
            action_log.append(row)

    # Make cmds
#    max_step = 1000
#    cmdSeq = np.zeros([max_step,5])
#    for i in range(0,max_step):
#        cmdSeq[i] = [-0.2, 0.4, 0.6, 0.7, 0]

    game = Simulator(SCREEN_WIDTH, SCREEN_HEIGHT, action_log)


if __name__ == "__main__":
    main()
