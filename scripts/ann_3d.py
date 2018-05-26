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

NUM_PTS = 200

'''
Creates and saves contour plot of ANN output
net_index: 0 for angle, 1 for transx, 2 for transy
num_episodes: number of episodes trained
Supply the ann by one of these arguments:
    model_path: file path to model.ckpt
    ann: already loaded ann
'''
def plotANN(net_index, num_episodes = 0, model_path=None, ann_in=None):
    # Time the function
    start_time =  timer()

    action_dim = 1
    action_space_high = 2.0

    # Set parameters depending on which net
    if (net_index == 0):
        state_dim = 3
        param1_max = np.pi * 2
        param2_max = 25
        if (num_episodes):
            plot_title = 'Angle Controller Output, %d Episodes' % num_episodes
        else:
            plot_title = 'Angle Controller Output'
        x_label = 'Theta (rad)'
        y_label = 'Theta dot (rad/s)'
    elif (net_index == 1):
        state_dim = 2
        param1_max = 1200
        param2_max = 1000
        if (num_episodes):
            plot_title = 'X Translation Controller Output, %d Episodes' % num_episodes
        else:
            plot_title = 'X Translation Controller Output'
        x_label = 'X (mm)'
        y_label = 'X dot (mm/s)'
    elif (net_index == 2):
        state_dim = 2
        param1_max = 600
        param2_max = 1000
        if (num_episodes):
            plot_title = 'Y Translation Controller Output, %d Episodes' % num_episodes
        else:
            plot_title = 'Y Translation Controller Output'
        x_label = 'Y (mm)'
        y_label = 'Y dot (mm/s)'

    if (ann_in == None):
        # Load the selected ANN
        my_ann = ann(model_path, state_dim = state_dim, action_dim = action_dim, action_space_high = action_space_high)
    else:
        my_ann = ann_in

    # Generate linearly spaced sample points
    param1_array = np.linspace(-param1_max, param1_max, NUM_PTS, endpoint=True)
    param2_array = np.linspace(-param2_max, param2_max, NUM_PTS, endpoint=True)

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

            # Get ANN prediction
            u = np.clip(my_ann.predict(obs), -2, 2)

            # Save 3D point
            Z[i][j] = u


    # Generate 3D plot
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
    filepath = os.path.join(os.getcwd(), "figures/%d_%d.pdf" % (net_index, num_episodes))
    directory = os.path.dirname(filepath)
    if not os.path.exists(directory):
        os.makedirs(directory)
    fig.savefig(filepath, bbox_inches='tight')
    end_time =  timer()
    print("Figure saved to %s. Time elapsed: %fs." % (filepath, end_time - start_time))

if __name__ == '__main__':
    net_index = 1
    #model_path = './trained-models/models-angle/model.ckpt'
    model_path = './trained-models-sim/models-transx/model.ckpt'
    num_episodes = 1
    plotANN(net_index, num_episodes, model_path)
