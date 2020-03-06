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
from geometry_msgs.msg import PoseStamped,Point
# Path.
from nav_msgs.msg import Path
# For pose information.
from tf.transformations import euler_from_quaternion
# For converting laser to point cloud
import laser_geometry.laser_geometry as lg
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

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

ROBOT_NAME = 'tb3_0'


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
        self._points=None

    def callback(self, msg):
        # print(msg)
        # Helper for angles.
        def _within(x, a, b):
            pi2 = np.pi * 2.
            x %= pi2
            a %= pi2
            b %= pi2
            if a < b:
                return a <= x and x <= b
            return a <= x or x <= b
        # TODO: write a cpp file instead to use the "high fidelity conversion"
        # https://answers.ros.org/question/11232/how-to-turn-laser-scan-to-point-cloud-map/
        # pc2_msg = self._lp.projectLaser(msg)
        # print(len(msg.ranges))
        # self._points = np.array(list(pc2.read_points(pc2_msg)))
        self._angles = np.arange(
            msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)

        # print("aaa",self._angles.shape, self._angles)
        # angles are counterclockwise, 0/2pi is straight ahead
        cone_left = np.pi/4
        cone_right = np.pi/4
        # limit field of view, only consider points close enough
        cone = np.where((self._angles > (2*np.pi - cone_right)) | (self._angles < cone_left) | (self._measurements < 3))
        self._measurements = np.array(msg.ranges)

        self.cone_measurements = self._measurements[cone]
        self.cone_angles = self._angles[cone]

        # print(np.column_stack((self.cone_angles,self.cone_measurements)))
        # TODO: precalculate sin/cos for cone_angles and cache
        f = np.isfinite(self.cone_measurements)
        angles = np.transpose(np.vstack((np.cos(self.cone_angles[f]),np.sin(self.cone_angles[f]))))
        # points is array of points, shape (n,2), [[x1,y1],[x2,y2] ...]
        points = np.array([self.cone_measurements[f]]).transpose() * angles
        # print(points)

        # points = []

        # for i in range(len(self.cone_measurements)):
        #     if math.isnan(self.cone_measurements[i]) or math.isinf(self.cone_measurements[i]):
        #         continue
        #     theta = self.cone_angles[i]
        #     r = self.cone_measurements[i]
        #     points.append(np.array([r*np.cos(theta),r*np.sin(theta)]))
        # points = np.array(points)
        # print(points)
        self._points = points
        # print(msg, self._angles, sself._measurements)
        # Compute indices the first time.
        # if self._indices is None:
        #     self._indices = [[] for _ in range(len(self._angles))]
        #     for i, d in enumerate(msg.ranges):
        #         angle = msg.angle_min + i * msg.angle_increment
        #         for j, center_angle in enumerate(self._angles):
        #             if _within(angle, center_angle - self._width / 2., center_angle + self._width / 2.):
        #                 self._indices[j].append(i)

        # ranges = np.array(msg.ranges)
        # for i, idx in enumerate(self._indices):
        # We do not take the minimum range of the cone but the 10-th percentile for robustness.
        # self._measurements[i] = np.percentile(ranges[idx], 10)

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
    def centroid(self):
        # sumx = 0.
        # sumy = 0.
        # num = 0
        # for point in self.points:
        #     if not math.isnan(point[0]) and not math.isnan(point[1]):
        #         sumx += point[0]
        #         sumy += point[1]
        #         num += 1
        # if num == 0:
        #     print("No points in cloud, stopping")
        #     return np.array([0,0])
        if len(self.points) == 0:
            print("No points in cloud, stopping")
            return np.array([0,0])

        relative_centroid = np.mean(self.points,axis=0)[X:Y+1]
        print("centroid: ", relative_centroid)

        # Clustering
        km = KMeans(n_clusters=5).fit(self.points)
        print(km.cluster_centers_)
        db = DBSCAN(eps=0.1, metric='euclidean')
        db.fit(self.points)
        print("conponents:",db.components_)
        print("labels:",db.labels_)
        print(db.labels_.shape,self.points.shape, db.components_.shape)
        lab = db.labels_
        clusters = []
        for i in range(0, np.max(lab)+1):
            a = self.points[np.nonzero(lab == i)]
            clusters.append(a)
        clusters = np.array(clusters)
        # TODO: publish all these detected clusters on some other marker
        centroids = np.array([get_centroid(ps) for ps in clusters])
        print("c", centroids)
        # ac = AgglomerativeClustering(n_clusters=None, dist)
        # print(ac.cluster_centers_)
        if len(centroids) == 0:
            return np.array([0.,0.])
        if len(centroids) == 1:
            return centroids[0]
        else:
            pd = pairwise_distances(centroids)
            # get first index of the minimum non-zero distance
            i = np.argwhere(pd==np.min(pd[(pd>0.)]))[0]
            # take two centroids that are closest together - these are legs
            c1 = centroids[i[0]]
            c2 = centroids[i[1]]
            c3 = (c1+c2)/2
            return c3
        # a = 'occupancy_grid'
        # b = ROBOT_NAME+'/base_scan'
        # if self._tf.frameExists(a) and self._tf.frameExists(b):
        #     try:
        #         t = rospy.Time(0)
        #         trans, _ = self._tf.lookupTransform(
        #             '/' + a, '/' + b, t)
        #         print("t",trans)
        #         translation = np.array([trans[X], trans[Y]])
        #         print("translation ", translation)
        #         # TODO: might need to rotate as well - use euler_from quaternion and rotate?    
        #         centroid = np.array([sumx/num,-sumy/num]) + translation
        #         print("c",np.array([sumx/num,-sumy/num]),";",centroid)
        #         return centroid
        #     except Exception as e:
        #         print(e)
        # else:
        #     print('Unable to find:', self._tf.frameExists(
        #         a), self._tf.frameExists(b))
        #     return
def get_centroid(points):
    return np.mean(points,axis=0)[X:Y+1]

def get_follow_position(pose, laser):
    c = laser.centroid
    # If we are at most 20cm away from the "legs", stop
    print(np.linalg.norm(c), " away from centroid")
    if np.linalg.norm(c) < 0.2:
        print("close enough, stopping")
        return np.array([0.,0.])
    return laser.centroid
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

        # goal_reached = np.linalg.norm(slam.pose[:2] - goal.position) < .2
        # if goal_reached:
        #     print("Goal Reached")
        #     publisher.publish(stop_msg)
        #     rate_limiter.sleep()
        #     continue

        # Follow path using feedback linearization.
        position = np.array([
            slam.pose[X] + EPSILON * np.cos(slam.pose[YAW]),
            slam.pose[Y] + EPSILON * np.sin(slam.pose[YAW])], dtype=np.float32)
        # v = get_velocity(position, np.array(current_path, dtype=np.float32))
        # follow is relative to robot frame
        follow = get_follow_position(position, laser)
        
        goal_reached = np.linalg.norm(follow) < .4
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
        publisher.publish(vel_msg)


        # publish point that is being followed 
        marker = Marker()
        marker.header.frame_id = "/tb3_0/base_link"
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

        # Update plan every 1s.

        # time_since = current_time - previous_time
        # if current_path and time_since < 2.:
        #     rate_limiter.sleep()
        #     continue
        # previous_time = current_time

        # Run RRT.
        # start_node, final_node = rrt.rrt(
        #     slam.pose, follow_position, slam.occupancy_grid)
        # current_path = get_path(final_node)
        # if not current_path:
        #     print('Unable to reach goal position:', follow_position)

        # Publish path to RViz.
        # path_msg = Path()
        # path_msg.header.seq = frame_id
        # path_msg.header.stamp = rospy.Time.now()
        # path_msg.header.frame_id = 'map'
        # for u in current_path:
        #     pose_msg = PoseStamped()
        #     pose_msg.header.seq = frame_id
        #     pose_msg.header.stamp = path_msg.header.stamp
        #     pose_msg.header.frame_id = 'map'
        #     pose_msg.pose.position.x = u[X]
        #     pose_msg.pose.position.y = u[Y]
        #     path_msg.poses.append(pose_msg)
        # path_publisher.publish(path_msg)

        rate_limiter.sleep()
        frame_id += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs RRT navigation')
    args, unknown = parser.parse_known_args()
    try:
        run(args)
    except rospy.ROSInterruptException:
        pass
