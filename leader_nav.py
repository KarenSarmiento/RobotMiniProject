#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import os
import rospy
import sys
import math
from sklearn.cluster import KMeans, AgglomerativeClustering, DBSCAN
from sklearn.metrics import pairwise_distances
# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Occupancy grid.
from nav_msgs.msg import OccupancyGrid
# Position.
from tf import TransformListener
# Goal.
from geometry_msgs.msg import PoseStamped, Point
# Path.
from nav_msgs.msg import Path
# For pose information.
from tf.transformations import euler_from_quaternion
# For converting laser to point cloud
import laser_geometry.laser_geometry as lg
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from pykalman import KalmanFilter
import sensor_msgs.point_cloud2 as pc2
# Import the potential_field.py code rather than copy-pasting.
directory = os.path.join(os.path.dirname(
    os.path.realpath(__file__)), '.')
sys.path.insert(0, directory)
try:
    import rrt
except ImportError:
    raise ImportError(
        'Unable to import potential_field.py. Make sure this file is in "{}"'.format(directory))


SPEED = .2
EPSILON = .1

X = 0
Y = 1
YAW = 2

if len(sys.argv) >= 2:
    ROBOT_NAME = sys.argv[1]
else:
    ROBOT_NAME = "tb3_1"


def feedback_linearized(pose, velocity, epsilon):
    u = 0.  # [m/s]
    w = 0.  # [rad/s] going counter-clockwise.

    # MISSING: Implement feedback-linearization to follow the velocity
    # vector given as argument. Epsilon corresponds to the distance of
    # linearized point in front of the robot.
    u = velocity[X] * np.cos(pose[YAW]) + velocity[Y]*np.sin(pose[YAW])
    w = np.reciprocal(
        epsilon) * (-velocity[X]*np.sin(pose[YAW])+velocity[Y]*np.cos(pose[YAW]))
    return u, w


def normalize(vector):
    return vector / np.linalg.norm(vector)


def get_velocity(position, path_points):
    # print(path_points)
    v = np.zeros_like(position)
    # return v
    if len(path_points) == 0:
        return v
    # print(path_points)

    # Find smallest distance from position to path
    points = (sorted(((i, np.linalg.norm(position - x))
                      for i, x in enumerate(path_points)), key=lambda x: x[1]))
    i, d = points[0]
    if i >= len(path_points) - 2:
        i = len(path_points) - 2
    v = path_points[i+1] - position
    dir = normalize(v)
    mag = SPEED
    # v = path_points[i+1] - path_points[i]

    # MISSING: Return the velocity needed to follow the
    # path defined by path_points. Assume holonomicity of the
    # point located at position.
    return cap(dir*np.sqrt(mag), max_speed=SPEED)


def cap(v, max_speed):
    n = np.linalg.norm(v)
    if n > max_speed:
        return v / n * max_speed
    return v


class SLAM(object):
    def __init__(self):
        rospy.Subscriber('/'+ROBOT_NAME+'/map', OccupancyGrid, self.callback)
        self._tf = TransformListener()
        self._occupancy_grid = None
        self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)

    def callback(self, msg):
        values = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.width, msg.info.height))
        processed = np.empty_like(values)
        processed[:] = rrt.FREE
        processed[values < 0] = rrt.UNKNOWN
        processed[values > 50] = rrt.OCCUPIED
        processed = processed.T
        origin = [msg.info.origin.position.x, msg.info.origin.position.y, 0.]
        resolution = msg.info.resolution
        self._occupancy_grid = rrt.OccupancyGrid(processed, origin, resolution)

    def update(self):
        # Get pose w.r.t. map.
        a = 'occupancy_grid'
        b = ROBOT_NAME+'/base_link'
        if self._tf.frameExists(a) and self._tf.frameExists(b):
            try:
                t = rospy.Time(0)
                position, orientation = self._tf.lookupTransform(
                    '/' + a, '/' + b, t)
                self._pose[X] = position[X]
                self._pose[Y] = position[Y]
                _, _, self._pose[YAW] = euler_from_quaternion(orientation)
            except Exception as e:
                print(e)
        else:
            print('Unable to find:', self._tf.frameExists(
                a), self._tf.frameExists(b))

    @property
    def ready(self):
        return self._occupancy_grid is not None and not np.isnan(self._pose[0])

    @property
    def pose(self):
        return self._pose

    @property
    def occupancy_grid(self):
        return self._occupancy_grid


class GoalPose(object):
    def __init__(self):
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback)
        self._position = np.array([np.nan, np.nan], dtype=np.float32)

    def callback(self, msg):
        # The pose from RViz is with respect to the "map".
        self._position[X] = msg.pose.position.x
        self._position[Y] = msg.pose.position.y
        print('Received new goal position:', self._position)

    @property
    def ready(self):
        return not np.isnan(self._position[0])

    @property
    def position(self):
        return self._position


def get_path(final_node):
    # Construct path from RRT solution.
    if final_node is None:
        return []
    path_reversed = []
    path_reversed.append(final_node)
    while path_reversed[-1].parent is not None:
        path_reversed.append(path_reversed[-1].parent)
    path = list(reversed(path_reversed))
    # Put a point every 5 cm.
    distance = 0.05
    offset = 0.
    points_x = []
    points_y = []
    for u, v in zip(path, path[1:]):
        center, radius = rrt.find_circle(u, v)
        du = u.position - center
        theta1 = np.arctan2(du[1], du[0])
        dv = v.position - center
        theta2 = np.arctan2(dv[1], dv[0])
        # Check if the arc goes clockwise.
        clockwise = np.cross(u.direction, du).item() > 0.
        # Generate a point every 5cm apart.
        da = distance / radius
        offset_a = offset / radius
        if clockwise:
            da = -da
            offset_a = -offset_a
            if theta2 > theta1:
                theta2 -= 2. * np.pi
        else:
            if theta2 < theta1:
                theta2 += 2. * np.pi
        angles = np.arange(theta1 + offset_a, theta2, da)
        offset = distance - (theta2 - angles[-1]) * radius
        points_x.extend(center[X] + np.cos(angles) * radius)
        points_y.extend(center[Y] + np.sin(angles) * radius)
    return zip(points_x, points_y)


class SimpleLaser(object):
    def __init__(self, lp, name=""):
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
        self.centroids = None
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

        # angles are counterclockwise, 0/2pi is straight ahead
        cone_left = np.pi/3
        cone_right = np.pi/3
        # limit field of view, only consider points close enough
        self._measurements = np.array(msg.ranges)
        cone = np.where(((self._angles > (2*np.pi - cone_right))
                         | (self._angles < cone_left)) & (self._measurements < 2.5))

        self.cone_measurements = self._measurements[cone]
        self.cone_angles = self._angles[cone]
        # TODO: precalculate sin/cos for cone_angles and cache
        f = np.isfinite(self.cone_measurements)
        angles = np.transpose(
            np.vstack((np.cos(self.cone_angles[f]), np.sin(self.cone_angles[f]))))
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
        return self._points

    @property
    def centroid(self, position):
        if len(self.points) == 0:
            print("No points in cloud, stopping")
            return np.array([0, 0])
        
        # Simple centroid - not using
        relative_centroid = np.mean(self.points, axis=0)[X:Y+1]
        
        # Will be None on the first step since we have no centroids
        self.last_centroids = self.centroids



        # Clustering
        current_points = self.points.copy()
        # Assume 10cm diameter legs
        db = DBSCAN(eps=0.1, metric='euclidean').fit(current_points)
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
        # publish_points("/centroids", centroids)
        
        # offset by slam.position to get coordinate in slam frame
        # TODO: do a tf transform instead?
        self.centroids = np.array([c + position for c in centroids])
        
        
        self.kalman_predict_centroids = [] # some function of the last centroids


        for c in self.centroids:
            min = float('inf')
            i_min = 0
            # get closest centroid to prediction
            for i,x in enumerate(self.kalman_predict_centroids):
                d = np.abs(np.linalg.norm(c-x))
                if d<min:
                    min = d
                    i_min = i
            # If cluster has jumped too far (i.e. the cluster being predicted has disappeared), add as new centroid
            if min < .5:
                self.cluster_map[self.next_id] = c
                self.next_id += 1
            # lc is the last centroid corresponding to the prediction closest to c
            lc = self.last_centroids[self.kalman_predict_centroids.index(min)]
            # update the centroid in the map of clusters
            id = self.cluster_map.keys()[self.cluster_map.values().index(lc)]
            self.cluster_map[id] = self.centroids[i_min]

        if self.kf is None:
            self.kf = KalmanFilter(transition_matrices = [[1, 1], [0, 1]], observation_matrices = [[0.1, 0.5], [-0.3, 0.0]])
            self.kf = self.kf.em(centroids, n_iter=5)


        if len(centroids) == 0:
            c = np.array([0., 0.])
        if len(centroids) == 1:
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
            if np.abs(np.linalg.norm(c1-c2)) < .5: 
                c3 = (c1+c2)/2
                c = c3
            else:
                return np.array([0.,0.])
        print("centroid: ", c)
        return c

def get_centroid(points):
    return np.mean(points, axis=0)[X:Y+1]


def get_follow_position(pose, laser):
    c = laser.centroid
    # If we are at most 20cm away from the "legs", stop
    print(np.linalg.norm(c), " away from centroid")
    if not np.isfinite(np.linalg.norm(c)):
        print(c, " is nan")
        return np.array([0., 0])
    if np.linalg.norm(c) < 0.2:
        print("close enough, stopping")
        return np.array([0., 0.])
    return c
    # cone = np.where((laser.angles > (7*np.pi/4)) | (laser.angles < (np.pi/4)))
    # print("cone: ",cone)
    # cone_angles = laser.angles[cone]
    # cone_measures = laser.measurements[cone]
    # close = np.where(cone_measures < 2.5)
    # close_angles = cone_angles[close]
    # closest = np.argmin(cone_measures)
    # closest_dist = cone_measures[closest]
    # closest_angle = cone_angles[closest]

    # x = pose[X] + closest_dist * np.cos(closest_angle)
    # y = pose[Y] + closest_dist * np.sin(closest_angle)
    # position = np.array([x, y])

    # print("centroid: ",get_centroid(laser)," pose: ", pose)


def run(args):
    rospy.init_node('rrt_navigation')
    print("START")
    # Update control every 100 ms.
    rate_limiter = rospy.Rate(100)
    publisher = rospy.Publisher('/'+ROBOT_NAME+'/cmd_vel', Twist, queue_size=5)
    # path_publisher = rospy.Publisher(
    #     '/'+ROBOT_NAME+'/path', Path, queue_size=1)
    follow_point_publisher = rospy.Publisher('/follow', Marker, queue_size=1)
    centroids_publisher = rospy.Publisher('/centroids', Marker, queue_size=1)
    slam = SLAM()
    goal = GoalPose()
    lp = lg.LaserProjection()
    laser = SimpleLaser(lp, name=ROBOT_NAME)

    frame_id = 0
    current_path = []
    previous_time = rospy.Time.now().to_sec()

    # Stop moving message.
    stop_msg = Twist()
    stop_msg.linear.x = 0.
    stop_msg.angular.z = 0.

    # Make sure the robot is stopped.
    i = 0
    while i < 10 and not rospy.is_shutdown():
        publisher.publish(stop_msg)
        rate_limiter.sleep()
        i += 1
    print("START2")
    while not rospy.is_shutdown():
        slam.update()
        current_time = rospy.Time.now().to_sec()
        # Make sure all measurements are ready.
        # Get map and current position through SLAM:
        # > roslaunch exercises slam.launch
        if not slam.ready or not laser.ready:
            rate_limiter.sleep()
            continue

        # Follow path using feedback linearization.
        position = np.array([
            slam.pose[X] + EPSILON * np.cos(slam.pose[YAW]),
            slam.pose[Y] + EPSILON * np.sin(slam.pose[YAW])], dtype=np.float32)
        # v = get_velocity(position, np.array(current_path, dtype=np.float32))
        # follow is relative to robot frame
        follow = get_follow_position(position, laser)

        # 20cm away from object is good enough
        goal_reached = np.linalg.norm(follow) < .2
        if goal_reached:
            print("Goal Reached")
            publisher.publish(stop_msg)
            rate_limiter.sleep()
            continue

        v = cap(0.2*follow, SPEED)
        print("v: ", v)
        u, w = feedback_linearized(slam.pose, v, epsilon=EPSILON)
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        print(vel_msg)
        publisher.publish(vel_msg)

        # publish point that is being followed
        marker = Marker()
        marker.header.frame_id = "/"+ROBOT_NAME+"/base_link"
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.pose.orientation.w = 1
        f = Point()
        f.x = follow[X]
        f.y = follow[Y]
        marker.points = [f]
        t = rospy.Duration()
        marker.lifetime = t
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        follow_point_publisher.publish(marker)

        rate_limiter.sleep()
        frame_id += 1

def publish_points(topic,points):
    # points is np array of 2-D points
    marker = Marker()

    marker.header.frame_id = "/"+ROBOT_NAME+"/base_link"
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
    follow_point_publisher.publish(marker)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs RRT navigation')
    args, unknown = parser.parse_known_args()
    try:
        run(args)
    except rospy.ROSInterruptException:
        pass
