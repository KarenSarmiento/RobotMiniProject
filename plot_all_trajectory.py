from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import matplotlib.pylab as plt

import sys

if __name__ == '__main__':

    data = np.genfromtxt('/tmp/gazebo_exercise_tb3_0.txt', delimiter=',')
    data1 = np.genfromtxt('/tmp/gazebo_exercise_tb3_1.txt', delimiter=',')
    data2 = np.genfromtxt('/tmp/gazebo_exercise_tb3_2.txt', delimiter=',')
    print(data)
    print(data1)
    print(data2)
    plt.figure()
    plt.plot(data[:, 0], data[:, 1], 'r', label='leader')
    plt.plot(data1[:, 0], data1[:, 1], 'g', label='follower 1')
    plt.plot(data2[:, 0], data2[:, 1], 'g', label='follower 2')
    # For the localization exercise.
    if data.shape[1] == 6:
        plt.plot(data[:, 3], data[:, 4], 'g', label='estimated')
    # Cylinder.
    a = np.linspace(0., 2 * np.pi, 20)
    x = np.cos(a) * .05 + .0
    y = np.sin(a) * .05 + 2.
    plt.plot(x, y, 'k')

    a = np.linspace(0., 2 * np.pi, 20)
    x = np.cos(a) * .05 + 0.4
    y = np.sin(a) * .05 + 2.
    plt.plot(x, y, 'k')
    # Walls.
    plt.plot([-2, 2.], [-2, -2], 'k')
    plt.plot([-2, 2.], [2.5, 2.5], 'k')
    plt.plot([-2, -2], [-2, 2.5], 'k')
    plt.plot([2, 2.], [-2, 2.5], 'k')
    plt.axis('equal')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.xlim([-2.5, 2.5])
    plt.ylim([-2.5, 3.])

    if data.shape[1] == 6:
        plt.figure()
        error = np.linalg.norm(data[:, :2] - data[:, 3:5], axis=1)
        plt.plot(error, c='b', lw=2)
        plt.ylabel('Error [m]')
        plt.xlabel('Timestep')

    plt.show()
