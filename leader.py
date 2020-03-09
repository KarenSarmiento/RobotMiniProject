from robot import Robot
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import rospy
from geometry_msgs.msg import Twist
import numpy as np
from sklearn.metrics import pairwise_distances
import os
import sys
directory = os.path.join(os.path.dirname(
    os.path.realpath(__file__)), '.')
sys.path.insert(0, directory)


X = 0
Y = 1
YAW = 2
SPEED = 0.2
stop_msg = Twist()
stop_msg.linear.x = 0.
stop_msg.angular.z = 0.


class Leader(Robot):
    def __init__(self, name, rate_limiter):
        super(Leader, self).__init__(name, rate_limiter=rate_limiter)
        self._epsilon = 0.1
        self.leg_pub = rospy.Publisher(
            '/' + name + '/legs', Marker, queue_size=5)
        self.centroid_pub = rospy.Publisher(
            '/' + name + '/centroids', Marker, queue_size=5)
        self.follow = None

    def update_velocities(self):
        if not self.laser.ready:
            return
        position = np.array([
            self.slam.pose[X] + self.epsilon * np.cos(self.slam.pose[YAW]),
            self.slam.pose[Y] + self.epsilon * np.sin(self.slam.pose[YAW])], dtype=np.float32)

        # follow is relative to robot frame
        follow = self.find_legs(position)

        # 20cm away from object is good enough
        goal_reached = np.linalg.norm(follow) < .2
        if goal_reached or not np.isfinite(np.linalg.norm(follow)):
            self.publisher.publish(stop_msg)
            self.rate_limiter.sleep()
            return
        v = cap(0.2*follow, SPEED)
        u, w = self.feedback_linearized(
            self.slam.pose, v, epsilon=self.epsilon)

        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        self.publisher.publish(vel_msg)

    def find_legs(self, position):

        centroids = self.laser.centroids
        self.publish_markers(centroids, self.centroid_pub)

        print("centroids: ", centroids, len(centroids))
        if len(centroids) == 0:
            c = np.array([0., 0.])
        elif len(centroids) == 1:
            # TODO: don't move if only one cluster (one cluster => not legs)
            c = centroids[0]
        else:
            pd = pairwise_distances(centroids)
            # get first index of the minimum non-zero distance
            i = np.argwhere(pd == np.min(pd[(pd > 0.)]))[0]
            # take two centroids that are closest together - these are legs
            # only take 2 centroids that are less than some distance apart if there's more than one cluster
            c1 = centroids[i[0]]
            c2 = centroids[i[1]]
            if np.abs(np.linalg.norm(c1-c2)) < .75:
                c3 = (c1+c2)/2
                c = c3
            else:
                return np.array([0., 0.])
        self.follow = c
        return c

    def publish_leg(self):
        self.publish_markers(np.array([self.follow]), self.leg_pub)

    @property
    def epsilon(self):
        return self._epsilon


def cap(v, max_speed):
    n = np.linalg.norm(v)
    if n > max_speed:
        return v / n * max_speed
    return v
