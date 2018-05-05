#!/usr/local/bin/python3
#import numpy as np
#import robotsim
from robot import *
from timeit import default_timer as timer
import time

kUSE_SIM = 0
kUSE_ANN = 0
kUSE_SM = 0
kTUNE_PID = 1

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
            try:
                while True:
                    start = timer()
                    robot.updateSensorValue()
                    if kUSE_ANN:
                        robot.predict()
                        robot.execute()
                    if kUSE_SM:
                        robot.stateMachineCycle()

                    robot.calcU()
                    robot.execute()

                    robot.printSensorVals()
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
                if kTUNE_PID:
                    pass
                else:
                    raise KeyboardInterrupt

            x_des = int(input("\rEnter desired X: "))
            y_des = int(input("Enter desired Y: "))
            robot.state[6].push(x_des)
            robot.state[7].push(y_des)
    except KeyboardInterrupt:
        pass

    robot.close()


