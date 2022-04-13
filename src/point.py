import numpy as np
from math import sqrt

class Point():

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f"({self.x}, {self.y})"
    def close_enough(self, goal_loc):
        return abs(self.x - goal_loc.x < .1) and abs(self.y - goal_loc.y < .1)

    def distance(self, other_point):
        return sqrt(self.x ** 2 - other_point.x ** 2 + self.y ** 2 - other_point.y ** 2)
    def to_numpy(self):
        return np.array([self.x, self.y])
