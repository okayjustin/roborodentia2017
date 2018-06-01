#!/usr/local/bin/python3
"""
Top level function for controlling robot. Intended to run on the Raspberry
Pi but never fully implemented.

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
from timeit import default_timer as timer
import time

kUSE_SIM = 0
kUSE_ANN = 0
kUSE_PID = 0

# Initialize field area, -1 Left, 0 Center, 1 Right
kFIELD_AREA_INIT = 1

if __name__ == "__main__":
    robot = Robot(kUSE_SIM, kFIELD_AREA_INIT)
    if kUSE_SIM:
        robot.env.setWallCollision(True)

    if robot.openSerial():
        print("Failed to connect to robot. Quitting.")
        quit()
    if kUSE_ANN:
        print("Initializing neural nets...")
        robot.initializeNets()
    print("Initializing desired x, y, theta...")
    robot.initXYT()
    print("Ready to go!")

    try:
        while True:
            start = timer()
            robot.updateSensorValue()
            if kUSE_ANN:
                robot.predict()
                robot.execute()

            if kUSE_PID:
                robot.stateMachineCycle()
                robot.calcU()
                robot.execute()

           # robot.printSensorVals()
            end = timer()
#            print("Cycle time: %0.1f" % (1000*(end - start)))

            if kUSE_SIM:
                robot.incTime()
                robot.render()
                # Limit speed
                while (timer() < start + 0.050):
                    pass
            else:
                # Limit speed to 20 Hz
                while (timer() < start + 0.050):
                    pass
    except KeyboardInterrupt:
        pass

    robot.close()


