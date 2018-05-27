#!/usr/local/bin/python3
'''
This program plots the network output as a function of its two inputs in 3D scatter
Requires ANN to be 2 input, 1 output.
'''
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from timeit import default_timer as timer
import tensorflow as tf
from ann import ann

NUM_PTS = 100

'''
Creates and saves contour plot of ANN output
net_index: 0 for angle, 1 for transx, 2 for transy
num_episodes: number of episodes trained
Supply the ann by one of these arguments:
    ann: already loaded ann
'''
def plotANN(net_index, ann_in, num_episodes = 0, act_or_crit = 0):
    # Time the function
    start_time =  timer()

    # Set parameters depending on which net
    if (net_index == 0):
        state_dim = 3
        param1_max = np.pi * 2
        param2_max = 25
        net_name = 'Angle'
        x_label = 'Theta (rad)'
        y_label = 'Theta dot (rad/s)'
    elif (net_index == 1):
        state_dim = 2
        param1_max = 1200
        param2_max = 1000
        net_name = 'X Translation'
        x_label = 'X (mm)'
        y_label = 'X dot (mm/s)'
    elif (net_index == 2):
        state_dim = 2
        param1_max = 600
        param2_max = 1000
        net_name = 'Y Translation'
        x_label = 'Y (mm)'
        y_label = 'Y dot (mm/s)'

    # Generate figure title
    ac_name = 'Actor' if (act_or_crit == 0) else 'Critic'
    ep_name = "%d Episodes" % num_episodes if (num_episodes != 0) else ""
    plot_title = "%s %s Output %s" % (net_name, ac_name, ep_name)

    # Generate linearly spaced sample points
    param1_array = np.linspace(-param1_max, param1_max, NUM_PTS, endpoint=True)
    param2_array = np.linspace(-param2_max, param2_max, NUM_PTS, endpoint=True)
    action_array = np.linspace(-2, 2, 10, endpoint=True)

    # Generate the X and Y grids
    X, Y = np.meshgrid(param1_array, param2_array)

    # 3d points for plotting
    Z = np.zeros([NUM_PTS, NUM_PTS])

    # Loop through every pair of sample points
    for i in range(0, NUM_PTS):
        for j in range(0, NUM_PTS):
            # Generate observation
            if (net_index == 0):
                obs = np.array([np.cos(X[i][j]), np.sin(X[i][j]), Y[i][j]])
            else:
                obs = np.array([X[i][j], Y[i][j]])
            obs = np.reshape(obs, (1, state_dim))

            # Get ANN prediction
            if (act_or_crit == 0):
                u = np.clip(ann_in.predict(obs), -2, 2)[0]
            else:
                # Get max Q from Q network
                u = -99999
                for action in action_array:
                    u_test = ann_in.predict(obs, np.reshape(action, (1, 1)))[0][0]
                    if (u_test > u):
                        u = u_test

            # Save output
            Z[i][j] = u

    # Generate contour plot
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.contourf(X, Y, Z, 20, cmap='RdGy')
    #contours = ax.contour(X, Y, Z, 10, colors='black')
    #ax.clabel(contours, inline=True, fontsize=8)
    plt.colorbar();
    plt.title(plot_title)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    #plt.show()

    # Save figure
    filepath = os.path.join(os.getcwd(), "figures/%s%d_%d.pdf" % (ac_name, net_index, num_episodes))
    directory = os.path.dirname(filepath)
    if not os.path.exists(directory):
        os.makedirs(directory)
    fig.savefig(filepath, bbox_inches='tight')
    end_time =  timer()
    print("Figure saved to %s. Time elapsed: %fs." % (filepath, end_time - start_time))

if __name__ == '__main__':
    net_index = 1
    action_dim = 1
    action_space_high = 2.0

    # Set parameters depending on which net
    if (net_index == 0):
        state_dim = 3
    elif (net_index == 1):
        state_dim = 2
    elif (net_index == 2):
        state_dim = 2
    #model_path = './trained-models/models-angle/model.ckpt'
    model_path = './trained-models-sim/models-transx/model.ckpt'

    ann_in = ann(model_path, state_dim = state_dim, action_dim = action_dim, action_space_high = action_space_high)
    act_or_crit = 0
    num_episodes = 1
    plotANN(net_index, ann_in, num_episodes, act_or_crit)
