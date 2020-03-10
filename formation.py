#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from laser import Laser
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from follower import Follower
from leader import Leader
from slam import SLAM
import rospy
import numpy as np
import argparse
import os
import sys
directory = os.path.join(os.path.dirname(
    os.path.realpath(__file__)), '.')
sys.path.insert(0, directory)

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
# For groundtruth information.
X = 0
Y = 1
YAW = 2


def run(args):
    rospy.init_node('formation')

    # Update control every 100 ms.
    rate_limiter = rospy.Rate(100)

    leader = Leader("tb3_0", rate_limiter)
    follower_1 = Follower("tb3_1", desired_rel_pos=np.array(
        [-0.5, 0.35, 0.0]), rate_limiter=rate_limiter,
        des_d=0.4,
        des_psi=3*np.pi/4,
        leader=leader,
        laser_range=[np.pi, np.pi])
    follower_2 = Follower("tb3_2", desired_rel_pos=np.array(
        [-0.5, -0.35, 0.0]), rate_limiter=rate_limiter,
        des_d=0.4,
        des_psi=5*np.pi/4,
        leader=leader,
        laser_range=[np.pi, np.pi])

    while not rospy.is_shutdown():
        leader.update_velocities()
        leader.publish_leg()
        leader.slam.update()

        follower_1.formation_velocity()
        follower_2.formation_velocity()
        rate_limiter.sleep()


if __name__ == '__main__':
    args = None
    try:
        run(args)
    except rospy.ROSInterruptException:
        pass
