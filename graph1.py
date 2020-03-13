#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

l_poses = np.loadtxt('leader_poses')
leg_positions = np.loadtxt('leg_positions')
tms = np.loadtxt('time_vals')

# Times into seconds
tms /= 1000000000.0
dists = np.linalg.norm(leg_positions[:2], axis=0)


plt.plot(tms, dists, label="error")
plt.text(70, 0.4, "Change position\nof legs")
plt.legend()
plt.show()
