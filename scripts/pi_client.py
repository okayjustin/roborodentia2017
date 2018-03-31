#!/usr/local/bin/python3
import tensorflow as tf
import numpy as np
from ddpg import ActorNetwork


class ann(object):
    def __init__(self, model_path, state_dim, action_dim, action_space_high):
        self.sess = tf.Session()

        # Initialize actor networ
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


model_path = './results/models/model.ckpt'
state_dim = 3
action_dim = 1
action_space_high = 2
angle_ann = ann(model_path, state_dim, action_dim, action_space_high)

print(angle_ann.predict([1,0,0]))
