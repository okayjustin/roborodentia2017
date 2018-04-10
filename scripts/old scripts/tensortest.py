#!/usr/local/bin/python3

import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'

import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt

data_length = 1  # Amount of data to simulate in seconds
time_step = 0.01 # Time between samples in seconds
num_samples = int(data_length / time_step)
friction_constant = 28       # N/(m/s)
mass = 3                    # Kg
max_applied_force = 9 * 4.44822 # Newtons, 1 lb-force ~= 4.44822
throttle_ratio = 1
max_speed = 1.5             # m/s

# Initial conditions
applied_force = np.zeros(num_samples)
force = np.zeros(num_samples)
acceleration = np.zeros(num_samples)
velocity = np.zeros(num_samples)
position = np.zeros(num_samples)

# Simulate movement
for n in range(1,num_samples):
    applied_force[n] = throttle_ratio * max_applied_force
    force[n] = applied_force[n] - velocity[n-1] * friction_constant
    acceleration[n] = force[n] / mass
    velocity[n] = velocity[n-1] + acceleration[n] * time_step
    position[n] = position[n-1] + velocity[n] * time_step
t = np.arange(0, data_length, time_step)

# Simulate sensor output by adding noise
## Add random noise (gaussian, mean 0, stdev 1)
position_noise = position + 0.03 * np.random.randn(num_samples)
position_noise_with_bias = np.zeros([len(position_noise),4])
for i in range(4,len(position_noise)):
    position_noise_with_bias[i] = [1., position_noise[i], position_noise[i-2], position_noise[i-4]]
#    print(position_noise_with_bias[i])
position_noise_with_bias = np.array(position_noise_with_bias).astype(np.float32)

# Keep track of the loss at each iteration so we can chart it later
losses = []
# How many iterations to run our training
training_steps = 200
# The learning rate. Also known has the step size. This changes how far
# we move down the gradient toward lower error at each step. Too large
# jumps risk inaccuracy, too small slow the learning.
learning_rate = 100.0

# In TensorFlow, we need to run everything in the context of a session.
with tf.Session() as sess:
    # Set up all the tensors.
    # Our input layer is the x value and the bias node.
    input = tf.constant(position_noise_with_bias)
    # Our target is the y values. They need to be massaged to the right shape.
    target = tf.constant(np.transpose([position]).astype(np.float32))
    # Weights are a variable. They change every time through the loop.
    # Weights are initialized to random values (gaussian, mean 0, stdev 0.1)
    weights = tf.Variable(tf.random_normal([4, 1], 0, 0.1))

    # Set up all operations that will run in the loop.
    # For all x values, generate our estimate on all y given our current
    # weights. So, this is computing y = w1 * bias + w2 * x
    yhat = tf.matmul(input, weights)
    # Compute the error, which is just the difference between our
    # estimate of y and what y actually is.
    yerror = tf.subtract(yhat,target)
    # We are going to minimize the L2 loss. The L2 loss is the sum of the
    # squared error for all our estimates of y. This penalizes large errors
    # a lot, but small errors only a little.
    loss = tf.nn.l2_loss(yerror)

    # Perform gradient descent.
    # This essentially just updates weights, like weights -= grads * learning_rate
    # using the partial derivative of the loss with respect to the
    # weights. It's the direction we want to go to move toward lower error.
    update_weights = tf.train.AdadeltaOptimizer(learning_rate, rho=0.99, epsilon=1e-08).minimize(loss)

    # Initialize all the variables defined above.
    tf.global_variables_initializer().run()

    # At this point, we've defined all our tensors and run our initialization
    # operations. We've also set up the operations that will repeatedly be run
    # inside the training loop. All the training loop is going to do is
    # repeatedly call run, inducing the gradient descent operation, which has the effect of
    # repeatedly changing weights by a small amount in the direction (the
    # partial derivative or gradient) that will reduce the error (the L2 loss).
    for i in range(training_steps):
        # Repeatedly run the operations, updating the TensorFlow variable.
        sess.run(update_weights)

        # Here, we're keeping a history of the losses to plot later
        # so we can see the change in loss as training progresses.
        losses.append(loss.eval())

    # Training is done, get the final values for the charts
    yhat = np.transpose(yhat.eval())[0]

error_meas = np.sum(abs(position_noise - position))
error_nn = np.sum(abs(yhat - position))
print(error_meas)
print(error_nn)
# Show the results.
fig, (ax1, ax2) = plt.subplots(2, 1)
ax1.scatter(t, position, c="r", alpha=.5)
ax1.scatter(t, position_noise, c='b', alpha=.5)
ax1.scatter(t, yhat, c="g", alpha=.6)
ax1.legend(["True position","Measurement","NN output"])
ax2.plot(range(0, training_steps), losses)
ax2.set_ylabel("Loss")
ax2.set_xlabel("Training steps")
plt.show()

