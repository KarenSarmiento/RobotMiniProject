from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import rospy
from slam import SLAM
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from laser import Laser


# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
# For groundtruth information.
X = 0
Y = 1
YAW = 2


class Robot(object):
    def __init__(self, name, rate_limiter):
        self.pose_history = []
        self.publisher = rospy.Publisher(
            '/' + name + '/cmd_vel', Twist, queue_size=5)
        self.laser = Laser(name=name)
        self.slam = SLAM(name=name)
        self.name = name
        self.rate_limiter = rate_limiter
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

        return u, w*1.2

    def publish_markers(self, points, pub):
        if points is None or len(points) == 0:
            return
        marker = Marker()
        marker.header.frame_id = "/"+self.name+"/base_link"
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.pose.orientation.w = 1

        ps = []
        for p in points:
            f = Point()
            f.x = p[X]
            f.y = p[Y]
            ps.append(f)
        marker.points = ps

        t = rospy.Duration()
        marker.lifetime = t
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        pub.publish(marker)

        self.rate_limiter.sleep()
