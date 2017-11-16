import math
from collections import deque

'''
This class is useful for keeping track of the value of a variable as well as the
mean, variance, and standard deviation. Stats are calculated iteratively so it is not
necessary to retain all past values. The stats come in two flavors: total and windowed.
Total mean, variance, and standard deviation are computed over all past values of the
variable (unless you clear()). Windowed calculates over a fixed number of past values set
by period.

References:
    http://www.taylortree.com/2010/11/running-variance.html
    https://www.johndcook.com/blog/standard_deviation/
'''
class RunningStat:
    def __init__(self, period):
        self.period = period
        # Keep a FIFO of the past values
        self.vals = deque((self.period + 1) * [0.0], self.period + 1)
        self.n = 0
        self.total_mean = 0.0
        self.square = 0.0
        self.win_mean = 0.0
        self.powsumavg = 0.0

    # Clears the total stats calculation but has no effect on windowed stats
    def clear(self):
        self.n = 0

    def push(self, val):
        self.vals.appendleft(val)
        self.n = self.n + 1
        if (self.n == 1):
            self.total_mean = val
            self.square = 0.0
        else:
            new_mean = self.total_mean + (val - self.total_mean)/self.n
            self.square = self.square + (val - self.total_mean)*(val - new_mean)
            # set up for next iteration
            self.total_mean = new_mean

        self.win_mean = self.win_mean + ((val - self.vals[self.period]) / self.period)
        newamt = val
        oldamt = self.vals[self.period]
        self.powsumavg = self.powsumavg + (((newamt * newamt) - (oldamt * oldamt)) / self.period)

    def numDataValues(self):
        return self.n

    def curVal(self):
        return self.vals[0]

    def mean(self):
        return self.total_mean

    def winMean(self):
        return self.win_mean

    def var(self):
        return  self.square/(self.n - 1) if (self.n > 1) else 0.0

    def winVar(self):
        return (self.powsumavg * self.period - self.period * self.win_mean * self.win_mean) / self.period

    def stdDev(self):
        return math.sqrt(self.var())

    def winStdDev(self):
        return math.sqrt(self.winVar())
