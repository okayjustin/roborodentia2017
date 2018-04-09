#!/usr/local/bin/python3
#import tensorflow as tf
import numpy as np
#from ddpg import ActorNetwork
#import robotsim
from robot import *
from timeit import default_timer as timer
import time


class ann(object):
    def __init__(self, model_path, state_dim, action_dim, action_space_high):
        #ann_graph
        self.graph = tf.Graph()
        self.sess = tf.Session(graph=self.graph)

        with self.graph.as_default():
            action_bound = np.tile(np.transpose(action_space_high),[1,1])
            self.actor = ActorNetwork(self.sess, state_dim, action_dim, action_bound, 0, 0, 1)
            self.sess.run(tf.global_variables_initializer())

            # Load saved model
            saver = tf.train.Saver()
            saver.restore(self.sess, model_path)
            print("Model restored.")

    def predict(self, s):
        a = self.actor.predict(np.reshape(s, (1, self.actor.s_dim)))
        return a[0]

    def close(self):
        self.sess.close()


if __name__ == "__main__":
    robot = Robot()
    while True:
        #for i in range(0,10):
        start = timer()
        robot.updateSensorValue()
        end = timer()
#        print(end - start)
        robot.printSensorVals()
        time.sleep(0.05)

    robot.close()


