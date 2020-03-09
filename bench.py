#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

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
from pyquaternion import Quaternion

leader_poses = []
follower_1_poses = []
follower_2_poses = []
times = []

X = 0
Y = 1
YAW = 2

class GroundtruthPose(object):
    def __init__(self, name='turtlebot3_burger'):
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        self._name = name

    def callback(self, msg):
        idx = [i for i, n in enumerate(msg.name) if n == self._name]
        if not idx:
            raise ValueError(
                'Specified name "{}" does not exist.'.format(self._name))
        idx = idx[0]
        self._pose[0] = msg.pose[idx].position.x
        self._pose[1] = msg.pose[idx].position.y
        _, _, yaw = euler_from_quaternion([
            msg.pose[idx].orientation.x,
            msg.pose[idx].orientation.y,
            msg.pose[idx].orientation.z,
            msg.pose[idx].orientation.w])
             
        self._pose[2] = yaw

    @property
    def ready(self):
        return not np.isnan(self._pose[0])

    @property
    def pose(self):
        return self._pose


def run(args):

    rospy.init_node('benchmarking')

    # Update measurements every 100 ms.
    rate_limiter = rospy.Rate(200)
 
    leader = GroundtruthPose("tb3_0")
    follower_1 = GroundtruthPose("tb3_1")
    follower_2 = GroundtruthPose("tb3_2")

    while not rospy.is_shutdown():
        while not leader.ready or not follower_1.ready or not follower_2.ready:
            pass
        global leader_poses
        leader_poses.append(leader.pose.copy())
        global follower_1_poses
        follower_1_poses.append(follower_1.pose.copy())
        global follower_2_poses
        follower_2_poses.append(follower_2.pose.copy())
        global times
        times.append(rospy.Time.now().to_nsec())
        rate_limiter.sleep()

def save_data():
  import matplotlib.pyplot as plt

  global leader_poses
  global follower_1_poses
  global follower_2_poses
  global times
  l_poses = np.transpose(np.array(leader_poses))
  f1_poses = np.transpose(np.array(follower_1_poses))
  f2_poses = np.transpose(np.array(follower_2_poses))
  tms = np.array(times)

  print("Saving values to files...")
  np.savetxt('leader_poses', l_poses)
  np.savetxt('follower_1_poses', f1_poses)
  np.savetxt('follower_2_poses', f2_poses)
  np.savetxt('time_vals', tms)
  print("Done.")

if __name__ == '__main__':
    args = None
    try:
        run(args)
    except rospy.ROSInterruptException:
        save_data()
        pass
