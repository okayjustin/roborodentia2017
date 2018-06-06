#!/usr/local/bin/python3
"""
Actor and Critic classes for use in DDPG.
ANN class for loading actor model for testing purposes.

Portions of the code have been copied from Patrick Emami's 
deep-rl Github repository (https://github.com/pemami4911/
deep-rl/blob/master/ddpg/ddpg.py) and is copyrighted under 
the terms of the MIT License.

The MIT License (MIT)

Copyright (c) 2018 Justin Ng
Copyright (c) 2016 Patrick E.

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

import tensorflow as tf
import numpy as np
import tflearn

ACTOR_L1_NODES = 400
ACTOR_L2_NODES = 300
CRITIC_L1_NODES = 400
CRITIC_L2_NODES = 300

# Standalone, non-training network using model loaded from file
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
        a = self.actor.predict(s)
        return a

    def close(self):
        self.sess.close()


# ===========================
#   Actor and Critic DNNs
# ===========================

class ActorNetwork(object):
    """
    Input to the network is the state, output is the action
    under a deterministic policy.

    The output layer activation is a tanh to keep the action
    between -action_bound and action_bound
    """

    def __init__(self, sess, state_dim, action_dim, action_bound, learning_rate, tau, batch_size):
        self.sess = sess
        self.s_dim = state_dim
        self.a_dim = action_dim
        self.action_bound = action_bound
        self.learning_rate = learning_rate
        self.tau = tau
        self.batch_size = batch_size

        # Actor Network
        self.inputs, self.out, self.scaled_out = self.create_actor_network()

        self.network_params = tf.trainable_variables()

        # Target Network
        self.target_inputs, self.target_out, self.target_scaled_out = self.create_actor_network()

        self.target_network_params = tf.trainable_variables()[
            len(self.network_params):]

        # Op for periodically updating target network with online network
        # weights
        self.update_target_network_params = [self.target_network_params[i].assign( \
                tf.multiply(self.network_params[i], self.tau) + \
                tf.multiply(self.target_network_params[i], 1. - self.tau))
                for i in range(len(self.target_network_params))]

        # This gradient will be provided by the critic network
        self.action_gradient = tf.placeholder(tf.float32, [None, self.a_dim])

        # Combine the gradients here
        self.unnormalized_actor_gradients = tf.gradients(
            self.scaled_out, self.network_params, -self.action_gradient)
        self.actor_gradients = list(map(lambda x: tf.div(x, self.batch_size), 
            self.unnormalized_actor_gradients))

        # Optimization Op
        self.optimize = tf.train.AdamOptimizer(self.learning_rate).\
            apply_gradients(zip(self.actor_gradients, self.network_params))

        self.num_trainable_vars = len(
            self.network_params) + len(self.target_network_params)

    def create_actor_network(self):
        inputs = tflearn.input_data(shape=[None, self.s_dim], name='ActorInputs')
        net = tflearn.layers.normalization.batch_normalization(inputs, name='ActorBatchNorm0Net')
        net = tflearn.fully_connected(net, ACTOR_L1_NODES, name='ActorInputsNet')
        # inputs = tflearn.input_data(shape=[None, self.s_dim], name='ActorInputs')
        # net = tflearn.fully_connected(inputs, ACTOR_L1_NODES, name='ActorInputsNet')
        
        net = tflearn.activations.relu(net)
        net = tflearn.layers.normalization.batch_normalization(net, name='ActorBatchNorm1Net')
        net = tflearn.fully_connected(net, ACTOR_L2_NODES, name='ActorNetNet')
        net = tflearn.activations.relu(net)
        net = tflearn.layers.normalization.batch_normalization(net, name='ActorBatchNorm2Net')
        # Final layer weights are init to Uniform[-3e-3, 3e-3]
        w_init = tflearn.initializations.uniform(minval=-0.003, maxval=0.003)
        out = tflearn.fully_connected(
            net, self.a_dim, activation='tanh', weights_init=w_init, name='ActorOutNet')
        # Scale output to -action_bound to action_bound
        scaled_out = tf.multiply(out, self.action_bound)
        return inputs, out, scaled_out

    def train(self, inputs, a_gradient):
        self.sess.run(self.optimize, feed_dict={
            self.inputs: inputs,
            self.action_gradient: a_gradient
        })

    def predict(self, inputs):
        return self.sess.run(self.scaled_out, feed_dict={
            self.inputs: inputs
        })

    def predict_target(self, inputs):
        return self.sess.run(self.target_scaled_out, feed_dict={
            self.target_inputs: inputs
        })

    def update_target_network(self):
        self.sess.run(self.update_target_network_params)

    def get_num_trainable_vars(self):
        return self.num_trainable_vars


class CriticNetwork(object):
    """
    Input to the network is the state and action, output is Q(s,a).
    The action must be obtained from the output of the Actor network.

    """

    def __init__(self, sess, state_dim, action_dim, learning_rate, tau, gamma, beta, num_actor_vars):
        self.sess = sess
        self.s_dim = state_dim
        self.a_dim = action_dim
        self.learning_rate = learning_rate
        self.tau = tau
        self.gamma = gamma
        self.beta = beta

        # Create the critic network
        self.inputs, self.action, self.out = self.create_critic_network()

        self.network_params = tf.trainable_variables()[num_actor_vars:]
        
        # Get network weights only for L2 weight decay (no biases)
        self.network_weights = [v for v in self.network_params if 'W' in v.name]

        # Target Network
        self.target_inputs, self.target_action, self.target_out = self.create_critic_network()

        self.target_network_params = tf.trainable_variables()[(len(self.network_params) + num_actor_vars):]

        # Op for periodically updating target network with online network
        # weights with regularization
        self.update_target_network_params = [self.target_network_params[i].assign( \
            tf.multiply(self.network_params[i], self.tau) + \
            tf.multiply(self.target_network_params[i], 1. - self.tau)) \
                for i in range(len(self.target_network_params))]

        # Network target (y_i)
        self.predicted_q_value = tf.placeholder(tf.float32, [None, 1])

        # Define loss and optimization Op
        self.regularizers = tf.nn.l2_loss(self.network_weights[0]) \
            + tf.nn.l2_loss(self.network_weights[1]) \
            + tf.nn.l2_loss(self.network_weights[2]) \
            + tf.nn.l2_loss(self.network_weights[3])
        # self.loss = (tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(
        #     out_layer, tf_train_labels) +
        #     beta*tf.nn.l2_loss(hidden_weights) +
        #     beta*tf.nn.l2_loss(hidden_biases) +
        #     beta*tf.nn.l2_loss(out_weights) +
        #     beta*tf.nn.l2_loss(out_biases)))
        self.loss = tflearn.mean_square(self.predicted_q_value, self.out) + self.beta * self.regularizers
        self.optimize = tf.train.AdamOptimizer(
            self.learning_rate).minimize(self.loss)

        # Get the gradient of the net w.r.t. the action.
        # For each action in the minibatch (i.e., for each x in xs),
        # this will sum up the gradients of each critic output in the minibatch
        # w.r.t. that action. Each output is independent of all
        # actions except for one.
        self.action_grads = tf.gradients(self.out, self.action)

        self.num_trainable_vars = len(
            self.network_params) + len(self.target_network_params)


    def create_critic_network(self):
        inputs = tflearn.input_data(shape=[None, self.s_dim], name='CriticInputs')
        action = tflearn.input_data(shape=[None, self.a_dim], name='CriticAction')
        net = tflearn.layers.normalization.batch_normalization(inputs)
        net = tflearn.fully_connected(net, CRITIC_L1_NODES, name='CriticInputsNet')
        net = tflearn.activations.relu(net)

        # Add the action tensor in the 2nd hidden layer
        # Use two temp layers to get the corresponding weights and biases
        t1 = tflearn.fully_connected(net, CRITIC_L2_NODES, name='CriticNetT1')
        t2 = tflearn.fully_connected(action, CRITIC_L2_NODES, name='CriticActionT2')

        net = tflearn.activation(
            tf.matmul(net, t1.W) + tf.matmul(action, t2.W) + t2.b, activation='relu')

        # linear layer connected to 1 output representing Q(s,a)
        # Weights are init to Uniform[-3e-3, 3e-3]
        w_init = tflearn.initializations.uniform(minval=-0.003, maxval=0.003)
        out = tflearn.fully_connected(net,1, weights_init=w_init,name='CriticNetOut')
        return inputs, action, out

    def train(self, inputs, action, predicted_q_value):
        return self.sess.run([self.out, self.optimize], feed_dict={
            self.inputs: inputs,
            self.action: action,
            self.predicted_q_value: predicted_q_value
        })

    def predict(self, inputs, action):
        return self.sess.run(self.out, feed_dict={
            self.inputs: inputs,
            self.action: action
        })

    def predict_target(self, inputs, action):
        return self.sess.run(self.target_out, feed_dict={
            self.target_inputs: inputs,
            self.target_action: action
        })

    def action_gradients(self, inputs, actions):
        return self.sess.run(self.action_grads, feed_dict={
            self.inputs: inputs,
            self.action: actions
        })

    def update_target_network(self):
        self.sess.run(self.update_target_network_params)
