import math
import numpy as np
from collections import deque
from itertools import islice

'''
This class is useful for keeping track of the value of a variable as well as the
mean, variance, and standard deviation. Stats are calculated iteratively so it is not
necessary to retain all past values. The stats come in two flavors: total and windowed.
Total mean, variance, and standard deviation are computed over all past values of the
variable (unless you clear()). Windowed calculates over a fixed number of past values set
by window. The median is calculated over a separate window defined by median_window.

References:
    http://www.taylortree.com/2010/11/running-variance.html
    https://www.johndcook.com/blog/standard_deviation/
'''
class RunningStat:
    def __init__(self, window, median_window = 30):
        self.window = window
        self.median_window = median_window
        # Keep a FIFO of the past values
        self.vals = deque((self.window) * [0.0], self.window)
        self.n = 0
        self.mean = 0.0
        self.square = 0.0
        self.win_mean = 0.0
        self.powsumavg = 0.0

    # Clears the total stats calculation but has no effect on windowed stats
    def clear(self):
        self.n = 0

    def push(self, val):
        self.n = self.n + 1
        if (self.n == 1):
            self.mean = val
            self.square = 0.0
        else:
            new_mean = self.mean + (val - self.mean)/self.n
            self.square = self.square + (val - self.mean)*(val - new_mean)
            # set up for next iteration
            self.mean = new_mean

        self.win_mean = self.win_mean + ((val - self.vals[self.window - 1]) / self.window)
        newamt = val
        oldamt = self.vals[self.window - 1]
        self.powsumavg = self.powsumavg + (((newamt * newamt) - (oldamt * oldamt)) / self.window)
        self.vals.appendleft(val)

    def numDataValues(self):
        return self.n

    def curVal(self):
        return self.vals[0]

    def mean(self):
        return self.mean

    def var(self):
        return  self.square/(self.n - 1) if (self.n > 1) else 0.0

    def stdDev(self):
        return math.sqrt(self.var())

    def winMean(self):
        return self.win_mean

    def winVar(self):
        return (self.powsumavg * self.window - self.window * self.win_mean * self.win_mean) / self.window

    def winStdDev(self):
        return math.sqrt(self.winVar())

    def winMedian(self):
        return np.median(list(islice(self.vals, 0, self.median_window)))
