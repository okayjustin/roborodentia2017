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

from replay_buffer import ReplayBuffer

kRENDER_EVERY = 1 # Render only every xth episode to speed up training

# Taken from https://github.com/openai/baselines/blob/master/baselines/ddpg/noise.py, which is
# based on http://math.stackexchange.com/questions/1287634/implementing-ornstein-uhlenbeck-in-matlab
class OrnsteinUhlenbeckActionNoise:
    def __init__(self, mu, sigma=0.3, theta=.15, dt=5e-2, x0=None):
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
#   Tensorflow Summary Ops
# ===========================


def build_summaries():
    episode_reward = tf.Variable(0.)
    tf.summary.scalar("Reward", episode_reward)
    episode_ave_max_q = tf.Variable(0.)
    tf.summary.scalar("Qmax Value", episode_ave_max_q)

    summary_vars = [episode_reward, episode_ave_max_q]
    summary_ops = tf.summary.merge_all()

    return summary_ops, summary_vars

# ===========================
#   Agent Training
# ===========================

def train(sess, env, args, actor, critic, actor_noise):

    # Set up summary Ops
    summary_ops, summary_vars = build_summaries()
    sess.run(tf.global_variables_initializer())

    saver = tf.train.Saver()
    writer = tf.summary.FileWriter(args['summary_dir'], sess.graph)

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

    try:
        for i in range(int(args['max_episodes'])):
            s = env.reset()

            ep_reward = 0
            ep_ave_max_q = 0

            robotsim_time = 0
            train_time = 0
            terminal = False
            for j in range(int(args['max_episode_len'])):

                if (args['render_env'] and i % kRENDER_EVERY == 0):
                    env.render()


                # Added exploration noise
                start = timer()
                a = actor.predict(np.reshape(s, (1, actor.s_dim))) + actor_noise()
                action = a[0]

                end = timer()
                train_time += end - start

                start = timer()

                s2, r, terminal, info = env.step(action)
                #print(s2)
                #print("Reward: %f" % r)
                #time.sleep(0.11)

                end = timer()
                robotsim_time += end - start

                start = timer()
                replay_buffer.add(np.reshape(s, (actor.s_dim,)), np.reshape(action, (actor.a_dim,)), r,
                                  terminal, np.reshape(s2, (actor.s_dim,)))

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
                    predicted_q_value, _ = critic.train(
                        s_batch, a_batch, np.reshape(y_i, (int(args['minibatch_size']), 1)))

                    ep_ave_max_q += np.amax(predicted_q_value)

                    # Update the actor policy using the sampled gradient
                    a_outs = actor.predict(s_batch)
                    grads = critic.action_gradients(s_batch, a_outs)
                    actor.train(s_batch, grads[0])

                    # Update target networks
                    actor.update_target_network()
                    critic.update_target_network()

                s = s2
                ep_reward += r
                end = timer()
                train_time += end - start

                # End of episode
                if terminal:
                    break

            # Print timeits
            total_time = robotsim_time + train_time
            # print("Robot: %fs (%d%%)   |      Train: %fs (%d%%)" %
            #     (robotsim_time, 100*robotsim_time/total_time, train_time, 100*train_time/total_time))
            summary_str = sess.run(summary_ops, feed_dict={
                summary_vars[0]: ep_reward,
                summary_vars[1]: ep_ave_max_q / float(j)
            })

            writer.add_summary(summary_str, i)
            writer.flush()

            # Determine network performance in actual test cases if the episode reward is low
            if (args['env'] == 'angle'):
                ep_reward_threshold = -30
            elif (args['env'] == 'transx'):
                ep_reward_threshold = -500
            elif (args['env'] == 'transy'):
                ep_reward_threshold = -350
            else:
                ep_reward_threshold = -30

            if (int(args['online'])):
                num_test_cases = 5
            else:
                num_test_cases = 50

            test_seed = int(np.random.uniform(1, 99999999))

            if (ep_reward > ep_reward_threshold):
                print("Testing network in %d cases..." % (num_test_cases))
                tf.set_random_seed(test_seed)
                env.seed(test_seed)
                test_total_reward = testNetworkPerformance(env, args, actor, num_test_cases)
                 # Save model temporarily
                save_path = saver.save(sess, "./results/models-temp/model.ckpt")

                # Restore the best model to test again
                try:
                    saver.restore(sess, "./results/models/model.ckpt")
                    tf.set_random_seed(test_seed)
                    env.seed(test_seed)
                    best_total_reward = testNetworkPerformance(env, args, actor, num_test_cases)
                except:
                    best_total_reward = -99999999999.

                # Restore the original model
                saver.restore(sess, "./results/models-temp/model.ckpt")

                # Save model if test reward increased
                if (test_total_reward > best_total_reward):
                    save_path = saver.save(sess, "./results/models/model.ckpt")
                    print("Model saved in path: %s" % save_path)
            else:
                test_total_reward = -999

            print('| Reward: {:d} | Test Reward: {:d} | Episode: {:d} | Qmax: {:.4f}'.format(int(ep_reward), \
                    int(test_total_reward), i, (ep_ave_max_q / float(j))))

    except KeyboardInterrupt:
        return


# Tests the current network
def testNetworkPerformance(env, args, actor, num_test_cases = 10, render = False):
    test_total_reward = 0.0

    # Test the network against random scenarios
    env.setWallCollision(True)
    for m in range(num_test_cases):
        s = env.reset()
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
def testModelPerformance(sess, env, args, actor, num_test_cases):
    sess.run(tf.global_variables_initializer())
    saver = tf.train.Saver()

    # Initialize target network weights
    actor.update_target_network()

    # Restore variables from disk.
    try:
        saver.restore(sess, args['model'])
        print("Model restored.")
    except:
        print("Can't restore model.")
        return

    # Test the network against random scenarios
    avg_test_reward = testNetworkPerformance(env, args, actor, num_test_cases, render=True)

    print('| Average reward: {:d}'.format(int(avg_test_reward)))
    return avg_test_reward

def writeActionLog(ep, action_log, ep_reward, ep_q, robot_init_state):
    filepath = os.path.join(os.getcwd(), "results/action_logs/%d_%d_%f.csv" % (ep, ep_reward, ep_q))
    directory = os.path.dirname(filepath)
    if not os.path.exists(directory):
        os.makedirs(directory)

    with open(filepath, 'w') as file:
        # Write init state
        first = True
        for state_var in robot_init_state:
            if (first):
                file.write("%f" % (state_var))
                first = False
            else:
                file.write(",%f" % (state_var))
        file.write("\n")

        # Write actions
        for action_set in action_log:
            first = True
            for action in action_set:
                if (first):
                    file.write("%f" % (action))
                    first = False
                else:
                    file.write(",%f" % (action))
            file.write("\n")

    return filepath

def main(args):

    if (args['env'] == 'angle'):
        env = robotsim.SimRobot(train = 'angle', online = int(args['online']))
    elif (args['env'] == 'transx'):
        env = robotsim.SimRobot(train = 'transx', online = int(args['online']))
    elif (args['env'] == 'transy'):
        env = robotsim.SimRobot(train = 'transy', online = int(args['online']))
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
        action_bound = np.tile(np.transpose(env.action_space.high),[int(args['minibatch_size']),1])

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
            testModelPerformance(sess, env, args, actor, num_test_cases=20)
        else:
            print("Beginning training...")
            train(sess, env, args, actor, critic, actor_noise)


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
    parser.add_argument('--random-seed', help='random seed for repeatability', default=1018)
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
