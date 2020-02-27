#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import rospy
import follow_leader as fl

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

X = 0
Y = 1
YAW = 2


def braitenberg(front, front_left, front_right, left, right):
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

    def _update_pose_history(self):
        if len(self.pose_history) % 10:
            with open('/tmp/gazebo_exercise_'+self.name+'.txt', 'a') as fp:
                fp.write('\n'.join(','.join(str(v) for v in p)
                                   for p in self.pose_history) + '\n')
                self.pose_history = []

class Leader(Robot):
    def __init__(self, name):
        super(Leader, self).__init__(name) 

    def update_velocities_braitenberg(self, rate_limiter):
        if not self.laser.ready or not self.groundtruth.ready:
            rate_limiter.sleep()
            return
        u, w = braitenberg(*self.laser.measurements)
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        self.publisher.publish(vel_msg)
        self.pose_history.append(self.groundtruth.pose)

        self._update_pose_history()


class Follower(Robot):
    def __init__(self, name, desired_rel_pos):
        super(Follower, self).__init__(name) 
        self._desired_rel_pos = desired_rel_pos
        self._epsilon = 0.2

    def formation_velocity(self, pose, leader_pose, rate_limiter):
        # Return if not ready.
        if not self.laser.ready or not self.groundtruth.ready:
            rate_limiter.sleep()
            return

        # Define position of holonomic point to be directly in front of the robot.
        hol_point_pos = np.array([
            pose[X] + self._epsilon * np.cos(pose[YAW]),
            pose[Y] + self._epsilon * np.sin(pose[YAW])
        ])
        
        # Get velocity for holonomic point.
        rotated_desired_rel_pos = _rotate(self._desired_rel_pos[:-1], leader_pose[YAW])
        v = self.get_velocity_using_p(hol_point_pos, leader_pose[:-1] + rotated_desired_rel_pos)
        # print("Got vel: {}".format(v))
        # print("Distance: {}".format(np.linalg.norm(v)))
        # print("Pose: {}".format(pose[:-1]))
        # print("hol_point_pos: {}".format(hol_point_pos))
        # print("Leader Pose: {}".format(leader_pose[:-1]))

        # Translate velocity to control inputs for non-holonomic robot.
        u, w = self.feedback_linearized(pose, v, self._epsilon)

        # Publish control inputs
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        self.publisher.publish(vel_msg)
        self.pose_history.append(self.groundtruth.pose)

        self._update_pose_history()

    def get_velocity_using_p(self, position, goal):
        # Stop moving if the goal is reached.
        if np.linalg.norm(position - goal) < .02:
            return np.zeros_like(position)

        # Calculate errors for P controller.
        error = goal - position

        # Return velocity.
        speed_sf = 1.0
        return speed_sf * error 

        
    def feedback_linearized(self, pose, velocity, epsilon):
        """Implement feedback-linearization.
        
        Args:
            pose: Current robot state.
            velocity: Vector [x,y] describing the x and y velocities of the holonomic point.
            epsilon: Distance of the linearised point in front of the robot.

        Returns:
            (u, w) which represents the velocity [m/s] and rotational velocity [rads/s] 
            (counter clockwise) of the robot.
        """
        u = velocity[X] * np.cos(pose[YAW]) + velocity[Y] * np.sin(pose[YAW])
        w = (-velocity[X] * np.sin(pose[YAW]) + velocity[Y] * np.cos(pose[YAW])) / epsilon

        return u, w

def _rotate(vector, angle, axis=[0.0, 0.0, 1.0]):
  return Quaternion(axis=axis,angle=angle).rotate(np.append(vector, [0.0]))[:2]

def run(args):
    rospy.init_node('obstacle_avoidance')

    # Update control every 100 ms.
    rate_limiter = rospy.Rate(100)

    leader = Leader("tb3_1")
    follower_1 = Follower("tb3_0", desired_rel_pos=np.array([0.0, 0.25, 0.0]))
    follower_2 = Follower("tb3_2", desired_rel_pos=np.array([0.0, -0.25, 0.0]))

    while not rospy.is_shutdown():
        # Make sure all measurements are ready.
        leader.update_velocities_braitenberg(rate_limiter=rate_limiter)
        follower_1.formation_velocity(
            pose=follower_1.groundtruth.pose,
            leader_pose=leader.groundtruth.pose,
            rate_limiter=rate_limiter)
        follower_2.formation_velocity(
            pose=follower_2.groundtruth.pose,
            leader_pose=leader.groundtruth.pose,
            rate_limiter=rate_limiter)

        rate_limiter.sleep()


if __name__ == '__main__':
    args = None
    try:
        run(args)
    except rospy.ROSInterruptException:
        pass
