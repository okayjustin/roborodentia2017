#!/usr/local/bin/python3
#import numpy as np
#import robotsim
from robot import *
from timeit import default_timer as timer
import time

kUSE_ANN = 1

if __name__ == "__main__":
    robot = Robot()
    if robot.openSerial():
        print("Failed to connect to robot. Quitting.")
        quit()

    if kUSE_ANN:
        print("Initializing neural nets...")
        robot.initializeNets()
        print("Zeroing theta...")
        robot.zeroTheta()
    print("Ready to go!")

    try:
        while True:
            start = timer()
            robot.updateSensorValue()
            if kUSE_ANN:
                robot.predict()
                robot.execute()

#            robot.printSensorVals()
            end = timer()
#            print("Cycle time: %0.1f" % (1000*(end - start)))

            # Limit speed to 20 Hz
            while (timer() < start + 0.050):
                pass
    except KeyboardInterrupt:
        pass

    robot.close()


