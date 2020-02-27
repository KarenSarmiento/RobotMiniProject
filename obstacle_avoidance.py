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


def get_velocities(front, front_left, front_right, left, right):
  sensors = np.array([1/left, 1/front_left, 1/front, 1/front_right, 1/right])
  angular_weights = np.array([-0.5, -1.0, 2.0, 1.0, 0.5])
  angular_scale_factor = 1.0/6.0
  w = np.dot(sensors, angular_weights) * angular_scale_factor
  vel_weights = np.array([0.0, 0.1, 0.5, 0.1, 0.0])
  vel_scale_factor = 1.0
  u = 1.0 - np.dot(sensors, vel_weights) * vel_scale_factor
  return u, w # ([m/s], [rad/s])

class SimpleLaser(object):
    def __init__(self, name=""):
        rospy.Subscriber('/'+name+'/scan', LaserScan, self.callback)
        self._angles = [0., np.pi / 4., -np.pi / 4., np.pi / 2., -np.pi / 2.]
        self._width = np.pi / 180. * 10.  # 10 degrees cone of view.
        self._measurements = [float('inf')] * len(self._angles)
        self._indices = None

    def callback(self, msg):
        # Helper for angles.
        def _within(x, a, b):
            pi2 = np.pi * 2.
            x %= pi2
            a %= pi2
            b %= pi2
            if a < b:
                return a <= x and x <= b
            return a <= x or x <= b

        # Compute indices the first time.
        if self._indices is None:
            self._indices = [[] for _ in range(len(self._angles))]
            for i, d in enumerate(msg.ranges):
                angle = msg.angle_min + i * msg.angle_increment
                for j, center_angle in enumerate(self._angles):
                    if _within(angle, center_angle - self._width / 2., center_angle + self._width / 2.):
                        self._indices[j].append(i)

        ranges = np.array(msg.ranges)
        for i, idx in enumerate(self._indices):
            # We do not take the minimum range of the cone but the 10-th percentile for robustness.
            self._measurements[i] = np.percentile(ranges[idx], 10)

    @property
    def ready(self):
        return not np.isnan(self._measurements[0])

    @property
    def measurements(self):
        return self._measurements


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


class Robot(object):

    def __init__(self, name):
        self.groundtruth = GroundtruthPose(name)
        self.pose_history = []
        self.publisher = rospy.Publisher('/' + name + '/cmd_vel', Twist, queue_size=5)
        self.laser = SimpleLaser(name=name)
        self.name = name
        with open('/tmp/gazebo_exercise_'+self.name+'.txt', 'w'):
            pass
    
    def update_velocities(self, rate_limiter):
        if not self.laser.ready or not self.groundtruth.ready:
            rate_limiter.sleep()
            return
        u, w = get_velocities(*self.laser.measurements)
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        self.publisher.publish(vel_msg)
        self.pose_history.append(self.groundtruth.pose)

        if len(self.pose_history) % 10:
            with open('/tmp/gazebo_exercise_'+self.name+'.txt', 'a') as fp:
                fp.write('\n'.join(','.join(str(v) for v in p)
                                   for p in self.pose_history) + '\n')
                self.pose_history = []


def run(args):
    rospy.init_node('obstacle_avoidance')
    # avoidance_method = globals()[args.mode]
    
    # Update control every 100 ms.
    rate_limiter = rospy.Rate(100)
    leader = Robot("tb3_0")
    follower_1 = Robot("tb3_1")
    follower_2 = Robot("tb3_2")

    while not rospy.is_shutdown():
        # Make sure all measurements are ready.
        leader.update_velocities(rate_limiter=rate_limiter)
        follower_1.update_velocities(rate_limiter=rate_limiter)
        follower_2.update_velocities(rate_limiter=rate_limiter)

        rate_limiter.sleep()


if __name__ == '__main__':
    # parser = argparse.ArgumentParser(description='Runs obstacle avoidance')
    # parser.add_argument('--mode', action='store', default='braitenberg',
    #                     help='Method.', choices=['braitenberg', 'rule_based'])
    # args, unknown = parser.parse_known_args()
    args = None
    try:
        run(args)
    except rospy.ROSInterruptException:
        pass
