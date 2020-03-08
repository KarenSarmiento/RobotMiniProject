#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from slam import SLAM
from leader import Leader
from follower import Follower
import argparse
import numpy as np
import rospy

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
# For groundtruth information.
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from laser import Laser
X = 0
Y = 1
YAW = 2


class Robot(object):
    def __init__(self, name):
        self.pose_history = []
        self.publisher = rospy.Publisher(
            '/' + name + '/cmd_vel', Twist, queue_size=5)
        self.laser = Laser(name=name)
        self.slam = SLAM(name=name)
        self.name = name
        with open('/tmp/gazebo_exercise_'+self.name+'.txt', 'w'):
            pass

    def _update_pose_history(self):
        if len(self.pose_history) % 10:
            with open('/tmp/gazebo_exercise_'+self.name+'.txt', 'a') as fp:
                fp.write('\n'.join(','.join(str(v) for v in p)
                                   for p in self.pose_history) + '\n')
                self.pose_history = []

    def feedback_linearized(self, pose, velocity, epsilon):
        u = velocity[X] * np.cos(pose[YAW]) + velocity[Y] * np.sin(pose[YAW])
        w = (-velocity[X] * np.sin(pose[YAW]) +
             velocity[Y] * np.cos(pose[YAW])) / epsilon

        return u, w


def run(args):
    rospy.init_node('formation')

    # Update control every 100 ms.
    rate_limiter = rospy.Rate(100)

    leader = Leader("tb3_1")
    follower_1 = Follower("tb3_0", desired_rel_pos=np.array([0.0, 0.25, 0.0]))
    follower_2 = Follower("tb3_2", desired_rel_pos=np.array([0.0, -0.25, 0.0]))

    while not rospy.is_shutdown():
        leader.update_velocities(rate_limiter=rate_limiter)

        follower_1.formation_velocity(rate_limiter=rate_limiter)

        follower_2.formation_velocity(rate_limiter=rate_limiter)

        rate_limiter.sleep()


if __name__ == '__main__':
    args = None
    try:
        run(args)
    except rospy.ROSInterruptException:
        pass
