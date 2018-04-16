import numpy as np


class MedianFilter(object):

    def __init__(self, window_size):
        self.window = np.zeros(window_size)

    def update(self, new_input):
        median = np.median(self.window)
        for i in range(len(self.window)):
            self.window[i+1] = self.window[i]
        self.window[0] = new_input
        return median

