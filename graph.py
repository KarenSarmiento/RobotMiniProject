#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

l_poses = np.loadtxt('leader_poses')
f1_poses = np.loadtxt('follower_1_poses')
f2_poses = np.loadtxt('follower_2_poses')
tms = np.loadtxt('time_vals')

desired_rel_pos_1 = np.array([0.0, 0.25, 0.0])
desired_rel_pos_2 = np.array([0.0, -0.25, 0.0])

target_rel_poses_1 = np.array([l_poses[0] * np.cos(l_poses[2])
        - l_poses[1] * np.sin(l_poses[2]),
        l_poses[0] * np.sin(l_poses[2])
        + l_poses[1] * np.cos(l_poses[2])])
target_poses_1 = np.array([ l_poses[0] + target_rel_poses_1[0],
        l_poses[1] + target_rel_poses_1[1] ])

target_rel_poses_2 = np.array([l_poses[0] * np.cos(l_poses[2])
        - l_poses[1] * np.sin(l_poses[2]),
        l_poses[0] * np.sin(l_poses[2])
        + l_poses[1] * np.cos(l_poses[2])])
target_poses_2 = np.array([ l_poses[0] + target_rel_poses_2[0],
        l_poses[1] + target_rel_poses_2[1] ])

dists_1 = (f1_poses[:2] - target_poses_1)
dists_1 = np.sqrt(dists_1[0]**2 + dists_1[1]**2)

dists_2 = (f2_poses[:2] - target_poses_2)
dists_2 = np.sqrt(dists_2[0]**2 + dists_2[1]**2)

plt.plot(tms, dists_1, label="Follower 1 Error")
plt.plot(tms, dists_2, label="Follower 2 Error")
plt.legend()
plt.show()

