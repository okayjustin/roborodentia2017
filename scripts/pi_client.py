#!/usr/local/bin/python3
#import numpy as np
#import robotsim
from robot import *
from timeit import default_timer as timer
import time

if __name__ == "__main__":
    robot = Robot()
    if robot.openSerial():
        print("Failed to connect to robot. Quitting.")
        quit()

    print("Initializing neural nets...")
    robot.initializeNets()
    print("Zeroing theta...")
    robot.zeroTheta()
    print("Ready to go!")

    try:
        while True:
            start = timer()
            robot.updateSensorValue()

#            robot.printSensorVals()
            #end = timer()
            #print( 1000*(end - start))

            # Limit speed to 20 Hz
            while (timer() < start + 0.155):
                pass
    except KeyboardInterrupt:
        pass

    robot.close()


