import numpy as np
import os
import rospy
import sys
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import LaserScan
from tf import TransformListener
from sklearn.cluster import DBSCAN


class Laser(object):
    def __init__(self, lp=None, name=""):
        rospy.Subscriber('/'+name+'/scan', LaserScan, self.callback)
        # self._angles = [0., np.pi / 4., -np.pi / 4., np.pi / 2., -np.pi / 2.]
        n = 100
        self._angles = np.linspace(-np.pi/2, np.pi/2, n)
        self._width = np.pi / 180. * 10.  # 10 degrees cone of view.
        self._measurements = [float('inf')] * len(self._angles)
        self._indices = None
        self._lp = lp
        self._point_gen = None
        self._tf = TransformListener()
        self._points = None
        self.name = name
        self.kf = None
        self.cluster_map = {}
        self.next_id = 0

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
        self._angles = np.arange(
            msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)
        # print(msg.ranges)
        # angles are counterclockwise, 0/2pi is straight ahead
        cone_left = np.pi/3
        cone_right = np.pi/3
        # limit field of view, only consider points close enough
        self._measurements = np.array(msg.ranges)
        cone = np.where(((self._angles > (2*np.pi - cone_right))
                         | (self._angles < cone_left)) & (self._measurements < 3.5))

        self.cone_measurements = self._measurements[cone]
        self.cone_angles = self._angles[cone]
        # TODO: precalculate sin/cos for cone_angles and cache
        f = np.isfinite(self.cone_measurements)
        angles = np.transpose(
            np.vstack((np.cos(self.cone_angles[f]), -np.sin(self.cone_angles[f]))))
        # points is array of points, shape (n,2), [[x1,y1],[x2,y2] ...]
        points = np.array([self.cone_measurements[f]]).transpose() * angles
        self._points = points

    @property
    def ready(self):
        return not np.isnan(self._measurements[0])

    @property
    def measurements(self):
        return self._measurements

    @property
    def angles(self):
        return self._angles

    @property
    def points(self):
        # print("points: ", self._points)
        return self._points

    @property
    def centroids(self, cluster_size=0.1):
        if self.points is None or len(self.points) == 0:
            print("No points in cloud, stopping")
            return np.array([])

        # Clustering
        current_points = self.points.copy()
        # Assume 10cm diameter legs
        db = DBSCAN(eps=cluster_size, metric='euclidean',
                    min_samples=2).fit(current_points)
        lab = db.labels_
        clusters = []
        # Sometimes happens - i think points updates while this function runs sometimes?
        if current_points.shape[0] != lab.shape[0]:
            # If it does happen, just stop and it fixes on next iteration
            return np.array([0., 0.])
        for i in range(0, np.max(lab)+1):
            a = current_points[np.nonzero(lab == i)]
            clusters.append(a)
        clusters = np.array(clusters)

        # TODO: publish all these detected clusters on some other marker
        centroids = np.array([get_centroid(ps) for ps in clusters])
        print(self.name + " centroids: ", centroids)
        return centroids


def get_centroid(points):
    return np.mean(points, axis=0)[0:2]
