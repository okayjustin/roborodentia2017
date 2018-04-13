#!/usr/local/bin/python3
#import numpy as np
#import robotsim
from robot import *
from timeit import default_timer as timer
import time

kUSE_ANN = 0
kUSE_PID = 1

# Initialize field area, -1 Left, 0 Center, 1 Right
kFIELD_AREA_INIT = -1

if __name__ == "__main__":
    robot = Robot(kFIELD_AREA_INIT)
    if robot.openSerial():
        print("Failed to connect to robot. Quitting.")
        quit()

    if kUSE_ANN:
        print("Initializing neural nets...")
        robot.initializeNets()

    print("Zeroing theta...")
    robot.zeroTheta()
    print("Initializing desired x/y...")
    robot.initXY()
    print("Ready to go!")

    try:
        while True:
            start = timer()
            robot.updateSensorValue()
            if kUSE_ANN:
                robot.predict()
                robot.execute()

            if kUSE_PID:
                robot.calcU()
                robot.execute()

            robot.printSensorVals()
            end = timer()
#            print("Cycle time: %0.1f" % (1000*(end - start)))

            # Limit speed to 20 Hz
            while (timer() < start + 0.050):
                pass
    except KeyboardInterrupt:
        pass

    robot.close()


