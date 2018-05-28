#!/usr/local/bin/python3
"""
Implementation of DDPG - Deep Deterministic Policy Gradient

Algorithm and hyperparameter details can be found here:
    http://arxiv.org/pdf/1509.02971v2.pdf

The algorithm is tested on the Pendulum-v0 OpenAI gym task
and developed with tflearn + Tensorflow

Author: Patrick Emami

Usage:
    python3 ddpg.py

    Options:
    --env [angle | transx | transy]
    --online [0 | 1]
    --model './results/models/model.cpkt'
    --test [0 | 1]
"""
import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
import subprocess

import tensorflow as tf
from tensorflow.python.tools import inspect_checkpoint as chkp
import numpy as np
import gym
from gym import wrappers
import argparse
import pprint as pp
import robotsim
from timeit import default_timer as timer
import platform
import time
from ann import *
from ann_plot import *

from replay_buffer import ReplayBuffer

kRENDER_EVERY = 20 # Render only every xth episode to speed up training
kTEST_PERIOD_ONLINE = 20
kTEST_PERIOD_OFFLINE = 20
kNUM_TEST_CASES_ONLINE = 2
kNUM_TEST_CASES_OFFLINE = 40

# Taken from https://github.com/openai/baselines/blob/master/baselines/ddpg/noise.py, which is
# based on http://math.stackexchange.com/questions/1287634/implementing-ornstein-uhlenbeck-in-matlab
class OrnsteinUhlenbeckActionNoise:
    def __init__(self, mu, sigma=0.3, theta=.15, dt=0.05, x0=None):
        self.theta = theta
        self.mu = mu
        self.sigma = sigma
        self.dt = dt
        self.x0 = x0
        self.reset()

    def __call__(self):
        x = self.x_prev + self.theta * (self.mu - self.x_prev) * self.dt + \
                self.sigma * np.sqrt(self.dt) * np.random.normal(size=self.mu.shape)
        self.x_prev = x
        return x

    def reset(self):
        self.x_prev = self.x0 if self.x0 is not None else np.zeros_like(self.mu)

    def __repr__(self):
        return 'OrnsteinUhlenbeckActionNoise(mu={}, sigma={})'.format(self.mu, self.sigma)

# ===========================
#   Agent Training
# ===========================

def train(sess, env, args, actor, critic, actor_noise):
    sess.run(tf.global_variables_initializer())
    saver = tf.train.Saver()

    # Initialize target network weights
    actor.update_target_network()
    critic.update_target_network()

    # Restore variables from disk.
    if (args['model'] != ''):
        try:
            saver.restore(sess, args['model'])
            print("Model restored.")
        except:
            print("Can't restore model.")
            return

    # Initialize replay memory
    replay_buffer = ReplayBuffer(int(args['buffer_size']), int(args['random_seed']))

    # Set the number of test cases and how often to test
    if (int(args['online'])):
        num_test_cases = kNUM_TEST_CASES_ONLINE
        test_period = kTEST_PERIOD_ONLINE
    else:
        num_test_cases = kNUM_TEST_CASES_OFFLINE
        test_period = kTEST_PERIOD_OFFLINE

    # Directory to keep training results in
    sess_dir = time.strftime("%Y-%m-%d %H.%M.%S", time.gmtime())
    os.makedirs('./results/%s/' % (sess_dir) )

    # Start training
    for i in range(int(args['max_episodes'])):
        try:
            # Train an episode
            render = args['render_env'] and (i % kRENDER_EVERY == 0)
            ep_len, ep_reward, ep_ave_max_q = \
                    trainEpisode(env, args, actor, critic, actor_noise, replay_buffer, render)
            
            print('Ep: %d | Reward: %d | Qmax: %0.4f' % \
                    (i, int(ep_reward), ep_ave_max_q / float(ep_len)))

            # Test the network's performance
            if (i % test_period == 0):
                # Test the network and get the total reward
                print("Testing network in %d cases..." % (num_test_cases))
                test_reward = testNetworkPerformance(env, args, actor, num_test_cases)

                # Save network session
                filepath = "./results/%s/%s_%d_%d/model.ckpt" % (sess_dir, args['env'], i+1,int(test_reward))
                save_path = saver.save(sess, filepath)

                # Contour plots of ANN output
                if (args['env'] != 'all'):
                    plotANN(env.net_index, actor, i+1, 0)
                    plotANN(env.net_index, critic, i+1, 1)
            else:
                test_reward = None

            writeLog(sess_dir, args, i + 1, ep_reward, ep_ave_max_q, test_reward)
        except KeyboardInterrupt:
            if (int(args['online'])):
                env.halt()
            should_test = input("Do you want to test network (y/n(default))?: ")
            if (should_test == 'y'):

                compareNetworks(sess, saver, env, args, actor, i, num_test_cases)
                if (args['env'] != 'all'):
                    plotANN(env.net_index, actor, i+1, 0)
                    plotANN(env.net_index, critic, i+1, 1)

# Trains one episode of data
def trainEpisode(env, args, actor, critic, actor_noise, replay_buffer, render):
    s = env.reset()

    # Track the episode reward and average max q
    ep_reward = 0
    ep_ave_max_q = 0

    terminal = False
    for j in range(int(args['max_episode_len'])):

        if (render):
            env.render()

        # Predict action and add exploration noise
        a = actor.predict(np.reshape(s, (1, actor.s_dim))) + actor_noise()
        action = a[0]

        # Execute action in environment to change state
        s2, r, terminal, info = env.step(action)

        replay_buffer.add(np.reshape(s, (actor.s_dim,)), \
                np.reshape(action, (actor.a_dim,)), r, terminal, \
                np.reshape(s2, (actor.s_dim,)))

        # Keep adding experience to the memory until
        # there are at least minibatch size samples
        if replay_buffer.size() > int(args['minibatch_size']):
            s_batch, a_batch, r_batch, t_batch, s2_batch = \
                replay_buffer.sample_batch(int(args['minibatch_size']))

            # Calculate targets
            target_q = critic.predict_target(
                s2_batch, actor.predict_target(s2_batch))

            y_i = []
            for k in range(int(args['minibatch_size'])):
                if t_batch[k]:
                    y_i.append(r_batch[k])
                else:
                    y_i.append(r_batch[k] + critic.gamma * target_q[k])

            # Update the critic given the targets
            predicted_q_value, _ = critic.train( s_batch, a_batch, \
                    np.reshape(y_i, (int(args['minibatch_size']), 1)))

            ep_ave_max_q += np.amax(predicted_q_value)

            # Update the actor policy using the sampled gradient
            a_outs = actor.predict(s_batch)
            grads = critic.action_gradients(s_batch, a_outs)
            actor.train(s_batch, grads[0])

            # Update target networks
            actor.update_target_network()
            critic.update_target_network()

        # Update state for next step
        s = s2

        # Increment episode reward
        ep_reward += r

        # End of episode
        if terminal:
            break

    return (j, ep_reward, ep_ave_max_q)

# Tests the actor network against a number of random cases
def testNetworkPerformance(env, args, actor, num_test_cases = 10, render = False):
    test_total_reward = 0.0

    # Test the network against random scenarios
    env.setWallCollision(True)
    for m in range(num_test_cases + 1):
        s = env.reset(False, True, m, num_test_cases)
        ep_reward = 0.0
        for n in range(int(args['max_episode_len'])):
            if (args['render_env'] and render == True):
                env.render()

            # Choose action based on inputs
            a = actor.predict(np.reshape(s, (1, actor.s_dim)))
            action = a[0]
            # Execute action and get new state, reward
            s, r, terminal, info = env.step(action)
            ep_reward += r
            if terminal:
                break
        test_total_reward += ep_reward

    env.setWallCollision(False)
    # Return the average test reward
    return test_total_reward / (m+1)

# Tests the performance of an input model
def testModelPerformance(sess, env, args, actor, model, num_test_cases):
    sess.run(tf.global_variables_initializer())
    saver = tf.train.Saver()

    # Initialize target network weights
    actor.update_target_network()

    # Restore variables from disk.
    try:
        saver.restore(sess, model)
        print("Model restored.")
    except:
        print("Can't restore model.")
        return

    # Test the network against random scenarios
    avg_test_reward = testNetworkPerformance(env, args, actor, num_test_cases, render=True)

    print('| Average reward: {:d}'.format(int(avg_test_reward)))
    return avg_test_reward

def writeLog(sess_dir, args, ep, ep_reward, ep_q, test_reward=None):
    filepath = os.path.join(os.getcwd(), "results/%s/%s.csv" % (sess_dir, args['env']))
    directory = os.path.dirname(filepath)
    if not os.path.exists(directory):
        os.makedirs(directory)

    with open(filepath, 'a') as file:
        if (test_reward == None):
            file.write("%d,%f,%f,\n" % (ep, ep_reward, ep_q))
        else:
            file.write("%d,%f,%f,%f\n" % (ep, ep_reward, ep_q, test_reward))
       
    return filepath

def main(args):

    if ((args['env'] == 'angle') or (args['env'] == 'transx') or \
        (args['env'] == 'transy') or (args['env'] == 'all')):
        env = robotsim.SimRobot(train = args['env'], online = int(args['online']))
    else:
        env = gym.make(args['env'])

    graph = tf.Graph()
    sess = tf.Session(graph=graph)

    with graph.as_default():

        print("Setting random seed")
        np.random.seed(int(args['random_seed']))
        tf.set_random_seed(int(args['random_seed']))
        env.seed(int(args['random_seed']))

        state_dim = env.observation_space.shape[0]
        action_dim = env.action_space.shape[0]
        action_bound = np.tile(np.transpose(env.action_space.high),
                [int(args['minibatch_size']),1])

        # Ensure action bound is symmetric
        assert ((env.action_space.high == -env.action_space.low).all())

        print("Instantiating actor...")
        actor = ActorNetwork(sess, state_dim, action_dim, action_bound,
                             float(args['actor_lr']), float(args['tau']),
                             int(args['minibatch_size']))
#        for op in tf.get_default_graph().get_operations():
#            print(str(op.name))

        print("Instantiating critic...")
        critic = CriticNetwork(sess, state_dim, action_dim,
                               float(args['critic_lr']), float(args['tau']),
                               float(args['gamma']),
                               actor.get_num_trainable_vars())

        # Remove unused items in graph collection to remove warnings
        tf.get_default_graph().clear_collection('data_preprocessing')
        tf.get_default_graph().clear_collection('data_augmentation')

        actor_noise = OrnsteinUhlenbeckActionNoise(mu=np.zeros(action_dim), dt=env.dt)

        if int(args['test']) == 1:
            testModelPerformance(sess, env, args, actor, args['model'], num_test_cases=20)
        else:
            print("Beginning training...")
            try:
                train(sess, env, args, actor, critic, actor_noise)
            except KeyboardInterrupt:
                print("Quitting.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='provide arguments for DDPG agent')

    # agent parameters
    parser.add_argument('--actor-lr', help='actor network learning rate', default=0.0001) #0.0001
    parser.add_argument('--critic-lr', help='critic network learning rate', default=0.001) #=0.001
    parser.add_argument('--gamma', help='discount factor for critic updates', default=0.99) #0.99
    parser.add_argument('--tau', help='soft target update parameter', default=0.001) #0.001
    parser.add_argument('--buffer-size', help='max size of the replay buffer', default=1000000)
    parser.add_argument('--minibatch-size', help='size of minibatch for minibatch-SGD', default=64)

    # run parameters
    parser.add_argument('--env', help='choose the gym env- tested on {Robot}', default='angle')
    parser.add_argument('--online', help='choose the gym env- tested on {Robot}', default='0')
    parser.add_argument('--random-seed', help='random seed for repeatability', default=1337)
    parser.add_argument('--max-episodes', help='max num of episodes to do while training', default=50000)
    parser.add_argument('--max-episode-len', help='max length of 1 episode', default=1000)
    parser.add_argument('--render-env', help='render the gym env', action='store_true')
    parser.add_argument('--monitor-dir', help='directory for storing gym results', default='./results/gym_ddpg')
    parser.add_argument('--summary-dir', help='directory for storing tensorboard info', default='./results/tf_ddpg')

    parser.add_argument('--model', help='saved model to restore', default='')
    parser.add_argument('--test', help='test model', default=0)

    parser.set_defaults(render_env=True)

    args = vars(parser.parse_args())

    pp.pprint(args)

    main(args)
