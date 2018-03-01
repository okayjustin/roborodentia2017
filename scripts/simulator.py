#!/usr/local/bin/python3
"""
Simulates the robot, playfield, sensors
Units are in millimeters, degrees, and seconds (for time).
"""

import arcade
import os
import numpy as np
import math
import helper_funcs as hf

SPRITE_SCALING = 1
BOX_WIDTH = 16

FIELD_XMAX = 2438.4
FIELD_YMAX = 1219.2

SCREEN_WIDTH = 1600
SCREEN_HEIGHT = 1000
MM_PER_PIX = 1.6

MOVEMENT_SPEED = 5
TIME_STEP = 0.01 # seconds


"""
Simulates a robot with sensors. Robot coordinates are defined as cartesian
xy, origin at robot's center. +y is directly towards front side of robot and +x is
directly towards right side of robot. Theta is the angle of a radial vector where 0 degrees is
pointing directly towards the right side of the robot and 90 degrees is directly towards the front.
len_x = length of the robot left to right
len_y = length of the robot front to back
x: starting x position of the robot origin in field coordinates
y: starting y position of the robot origin in field coordinates
theta: starting angle of the robot origin in field coordinates
"""
class SimRobot():
    def __init__(self,x,y,len_x=315.0,len_y=275.0,theta=0.0):
        self.len_x = len_x
        self.len_y = len_y

        # Magnitude and angle of a line between the center and top right corner
        self.diag_angle = math.atan(len_y / len_x)
        self.diag_len = math.sqrt(pow(len_x,2) + pow(len_y,2)) / 2

        self.x = x
        self.y = y
        self.theta = theta

        # Converts wheel velocities to tranlational x,y, and rotational velocities
        self.max_wheel_w = 2*math.pi*4 # max wheel rotational velocity in rad/s
        r =  30.0   # Wheel radius in mm
        L1 = 119.35 # Half the distance between the centers of front and back wheels in mm
        L2 = 125.7  # Half the distance between the centers of left and right wheels in mm
        self.mecanum_xfer = (r/4) * np.array([[1,1,1,1],[-1,1,-1,1],\
                [1/(L1+L2),-1/(L1+L2),-1/(L1+L2),1/(L1+L2)]])

        # Initialize rangefinders
        self.rf1 = SimRangefinder(self, 0.0, self.len_y/2.0, 90.0)
        self.rf2 = SimRangefinder(self, -1 * self.len_x/2.0, 0.0, 180.0)
        self.rf3 = SimRangefinder(self, 0.0, -1 * len_y/2.0, 270.0)
        self.rf4 = SimRangefinder(self, self.len_x/2.0, 0.0, 0.0)

    """
    ctrl is a array with the various motor/launcher control inputs, valid range -1 to +1
    ctrl[0]: Front left motor, +1 rotates towards right
    ctrl[1]: Back left motor, +1 rotates towards right
    ctrl[2]: Back right motor, +1 rotates towards right
    ctrl[3]: Front right motor, +1 rotates towards right
    ctrl[4]: Launcher
    """
    def update(self,ctrl):
         # Wheel rotational velocities in degrees/sec
        wheel_w = self.max_wheel_w * np.array([[ctrl[0]],[ctrl[1]],[ctrl[2]],[ctrl[3]]])

        # Calculate robot frontwards, rightwards, and rotational velocities
        velocities = np.matmul(self.mecanum_xfer, wheel_w)
        right_vel = velocities[0][0]
        front_vel = velocities[1][0]
        rotation_vel = velocities[2][0]

        # Calculate the x and y velocity components
        self.theta = self.theta + (rotation_vel * TIME_STEP * 180 / math.pi)
        x_vel = right_vel * math.cos(math.radians(self.theta)) + front_vel * math.sin(math.radians(self.theta))
        y_vel = right_vel * math.sin(math.radians(self.theta)) + front_vel * math.cos(math.radians(self.theta))

        # Calculate the x and y spacing for collision detection
        # Calculate cos and sin of the angle
        angle = math.radians(self.theta % 180)
        cosval = math.cos(angle)

        if (cosval >= 0):
            x_space = abs(self.diag_len * math.cos(math.pi - self.diag_angle + angle))
            y_space = abs(self.diag_len * math.sin(self.diag_angle + angle))
        else:
            x_space = abs(self.diag_len * math.cos(self.diag_angle + angle))
            y_space = abs(self.diag_len * math.sin(math.pi - self.diag_angle + angle))

        self.x = hf.limitValue(self.x + x_vel * TIME_STEP,x_space,FIELD_XMAX - x_space)
        self.y = hf.limitValue(self.y + y_vel * TIME_STEP,y_space,FIELD_YMAX - y_space)

        self.updateRFs(0)

    def updateRFs(self,dummy):
        self.rf1.getMeas(self)
        self.rf2.getMeas(self)
        self.rf3.getMeas(self)
        self.rf4.getMeas(self)


"""
Simulates a rangefinder.
x: x-position in robot coordinates
y: y-position in robot coordinates
theta: orientation in robot coordinates
min_range: minimum measurement range
max_range: maximum measurement range
meas_period: amount of time between measurements
"""
class SimRangefinder():
    def __init__(self,robot,x,y,theta,max_range=1200.0,meas_period=0.033,meas_var=0.03):
        self.robot = robot
        self.theta = theta
        self.radius = math.sqrt(math.pow(x,2) + math.pow(y,2))
        rf_field_theta = (self.robot.theta+self.theta) * math.pi / 180.0
        self.dim = [self.robot.x + self.radius * math.cos(rf_field_theta),\
            self.robot.y + self.radius * math.sin(rf_field_theta), 0.0, 0.0]
        self.max_range = max_range
        self.meas_period = meas_period
        self.meas_var = meas_var
        self.meas = 0
        self.meas_x = 0
        self.meas_y = 0


    """
    Returns the measurement of the rangefinder in mm.
    robot: robot object
    """
    def getMeas(self, robot):
        self.robot = robot

        # Calculate the sensor's position and orientation in the field
        rf_field_theta = (self.robot.theta+self.theta) * math.pi / 180.0
        self.dim[0] = self.robot.x + self.radius * math.cos(rf_field_theta)
        self.dim[1] = self.robot.y + self.radius * math.sin(rf_field_theta)

        # Calculate cos and sin of the angle
        cosval = math.cos(rf_field_theta)
        sinval = math.sin(rf_field_theta)

        if cosval >= 0:
            self.meas_x = FIELD_XMAX - self.dim[0]
        else:
            self.meas_x = -1 * self.dim[0]
        if sinval >= 0:
            self.meas_y = FIELD_YMAX - self.dim[1]
        else:
            self.meas_y = -1 * self.dim[1]


        # Calculate the two possible measurements
        try:
            hx = self.meas_x / cosval # If the hypotenuse extends to a vertical wall
            hy = self.meas_y / sinval # If the hypotenuse extends to a horizontal wall

            # The correct measurement is the shorter one
            if (hx < hy):
                self.meas = hx
                self.dim[2] = self.dim[0] + self.meas_x
                self.dim[3] = self.dim[1] + self.meas * sinval
            else:
                self.meas = hy
                self.dim[2] = self.dim[0] + self.meas * cosval
                self.dim[3] = self.dim[1] + self.meas_y
        except ZeroDivisionError:
            if (cosval != 0):
                self.meas = self.meas_x / cosval
                self.dim[2] = self.dim[0] + self.meas_x
                self.dim[3] = self.dim[1] + self.meas * sinval
            else:
                self.meas = self.meas_y / sinval
                self.dim[2] = self.dim[0] + self.meas * cosval
                self.dim[3] = self.dim[1] + self.meas_y

        # Limit the sensor range to 10mm up to 1200mm
        if (self.meas > 1200):
            self.meas = 1200
        elif (self.meas < 10):
            self.meas = 10






class SimIMU():
    def __init__(self,x,y):
        return


class SimMicroSW():
    def __init__(self):
        return


class MyGame(arcade.Window):
    """ Main application class. """

    def __init__(self, width, height):
        """
        Initializer
        """
        super().__init__(width, height)

        # Set the working directory (where we expect to find files) to the same
        # directory this .py file is in. You can leave this out of your own
        # code, but it is needed to easily run the examples using "python -m"
        # as mentioned at the top of this program.
        file_path = os.path.dirname(os.path.abspath(__file__))
        os.chdir(file_path)

        # Sprite lists
        self.all_sprites_list = None
        self.coin_list = None

        # Set up the player
        self.score = 0
        self.player_sprite = None
        self.wall_list = None
        self.physics_engine = None

        self.robot = SimRobot(FIELD_XMAX / 2, FIELD_YMAX/2)

    def setup(self):
        """ Set up the game and initialize the variables. """
        # Sprite lists
        self.all_sprites_list = arcade.SpriteList()
        self.wall_list = arcade.SpriteList()

        # Offset values in pixels to move (0,0) coordinate into field area
        self.xoffset = (SCREEN_WIDTH/2) - FIELD_XMAX / (2 * MM_PER_PIX)
        self.yoffset = (SCREEN_HEIGHT/2) - FIELD_YMAX / (2 * MM_PER_PIX)

        # Set up the player
        self.score = 0
        self.time = 0
        self.player_sprite = arcade.Sprite("images/robot.png", SPRITE_SCALING)
        self.all_sprites_list.append(self.player_sprite)

        # Create walls of field
        wall = arcade.Sprite("images/boxCrate_1556.png", SPRITE_SCALING)
        wall.center_x = int(SCREEN_WIDTH / 2)
        wall.center_y = int((SCREEN_HEIGHT + FIELD_YMAX/MM_PER_PIX + BOX_WIDTH)/2)
        self.all_sprites_list.append(wall)
        self.wall_list.append(wall)

        wall = arcade.Sprite("images/boxCrate_1556.png", SPRITE_SCALING)
        wall.center_x = int(SCREEN_WIDTH / 2)
        wall.center_y = int((SCREEN_HEIGHT - FIELD_YMAX/MM_PER_PIX - BOX_WIDTH)/2)
        self.all_sprites_list.append(wall)
        self.wall_list.append(wall)

        wall = arcade.Sprite("images/boxCrate_794.png", SPRITE_SCALING)
        wall.center_x = int((SCREEN_WIDTH + FIELD_XMAX/MM_PER_PIX + BOX_WIDTH)/2)
        wall.center_y = int(SCREEN_HEIGHT / 2)
        self.all_sprites_list.append(wall)
        self.wall_list.append(wall)

        wall = arcade.Sprite("images/boxCrate_794.png", SPRITE_SCALING)
        wall.center_x = int((SCREEN_WIDTH - FIELD_XMAX/MM_PER_PIX - BOX_WIDTH)/2)
        wall.center_y = int(SCREEN_HEIGHT / 2)
        self.all_sprites_list.append(wall)
        self.wall_list.append(wall)

#        self.physics_engine = arcade.PhysicsEngineSimple(self.player_sprite, self.wall_list)

        #arcade.schedule(self.robot.updateRFs, 0.33)

        # Set the background color
        arcade.set_background_color(arcade.color.AMAZON)

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

        # Print info text
        arcade.draw_text("Time: %fs" % (self.time), 10, 50, arcade.color.BLACK, 12)
        arcade.draw_text("Rangefinder measurements: %8.2f %8.2f %8.2f %8.2f" %\
                (self.robot.rf1.meas, self.robot.rf2.meas, self.robot.rf3.meas, self.robot.rf4.meas),\
                10, 30, arcade.color.BLACK, 12)
        arcade.draw_text("Robot: x: %8.2f, y: %8.2f, theta: %8.2f)" % \
                (self.robot.x, self.robot.y, self.robot.theta), 10, 10, arcade.color.BLACK, 12)

    # Input array should be [x1,y1,x2,y2]
    def draw_line_mm(self,dim):
        arcade.draw_line(dim[0]/MM_PER_PIX + self.xoffset, dim[1]/MM_PER_PIX + self.yoffset, \
                dim[2]/MM_PER_PIX + self.xoffset, dim[3]/MM_PER_PIX + self.yoffset,
                arcade.color.WOOD_BROWN, 3)

    def on_key_press(self, key, modifiers):
        """Called whenever a key is pressed. """
        if key == arcade.key.UP:
            self.player_sprite.change_y = MOVEMENT_SPEED
        elif key == arcade.key.DOWN:
            self.player_sprite.change_y = -MOVEMENT_SPEED
        elif key == arcade.key.LEFT:
            self.player_sprite.change_x = -MOVEMENT_SPEED
        elif key == arcade.key.RIGHT:
            self.player_sprite.change_x = MOVEMENT_SPEED

    def on_key_release(self, key, modifiers):
        """Called when the user releases a key. """
        if key == arcade.key.UP or key == arcade.key.DOWN:
            self.player_sprite.change_y = 0
        elif key == arcade.key.LEFT or key == arcade.key.RIGHT:
            self.player_sprite.change_x = 0

    def update(self, delta_time):
        """ Movement and game logic """
        # Call update on all sprites (The sprites don't do much in this
        # example though.)
        #self.physics_engine.update()

        self.time += TIME_STEP

        # Next step of robot simulation
        self.robot.update([0.4, -0.4, -0.4, 0.4, 0])

        # Update graphics vars
        self.player_sprite.position[0] = self.robot.x/MM_PER_PIX + self.xoffset
        self.player_sprite.position[1] = self.robot.y/MM_PER_PIX + self.yoffset
        self.player_sprite.angle = self.robot.theta


def main():
    game = MyGame(SCREEN_WIDTH, SCREEN_HEIGHT)
    game.setup()
    arcade.run()


if __name__ == "__main__":
    main()
