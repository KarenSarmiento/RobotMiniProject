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
from sklearn.cluster import DBSCAN
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

def cap(v, max_speed):
    n = np.linalg.norm(v)
    if n > max_speed:
        return v / n * max_speed
    return v



def get_simple_centroid(points):
    # points in np array of N points, shape (N, 2)
    return np.mean(points, axis=0)[X:Y+1]

def get_centroids(laser):
    pass

def get_follow_position(pose, laser):
    c = laser.centroid
    
    if len(laser.points) == 0:
        print("No points in cloud, stopping")
        return np.array([0, 0])
    
    # Simple centroid - not using
    relative_centroid = np.mean(laser.points, axis=0)[X:Y+1]
    
    # Will be None on the first step since we have no centroids
    laser.last_centroids = laser.centroids

    # Clustering
    current_points = laser.points.copy()
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
    laser.centroids = np.array([c + position for c in centroids])
    
    
    laser.kalman_predict_centroids = [] # some function of the last centroids


    for c in laser.centroids:
        min = float('inf')
        i_min = 0
        # get closest centroid to prediction
        for i,x in enumerate(laser.kalman_predict_centroids):
            d = np.abs(np.linalg.norm(c-x))
            if d<min:
                min = d
                i_min = i
        # If cluster has jumped too far (i.e. the cluster being predicted has disappeared), add as new centroid
        if min < .5:
            laser.cluster_map[self.next_id] = c
            laser.next_id += 1
        # lc is the last centroid corresponding to the prediction closest to c
        lc = laser.last_centroids[laser.kalman_predict_centroids.index(min)]
        # update the centroid in the map of clusters
        id = laser.cluster_map.keys()[laser.cluster_map.values().index(lc)]
        laser.cluster_map[id] = laser.centroids[i_min]

    if laser.kf is laser:
        laser.kf = KalmanFilter(transition_matrices = [[1, 1], [0, 1]], observation_matrices = [[0.1, 0.5], [-0.3, 0.0]])
        laser.kf = laser.kf.em(centroids, n_iter=5)


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
    
    # If we are at most 20cm away from the "legs", stop
    if not np.isfinite(np.linalg.norm(c)):
        print(c, " is nan")
        return np.array([0., 0])
    if np.linalg.norm(c) < 0.2:
        print("close enough, stopping")
    print(np.linalg.norm(c), " away from centroid")
        return np.array([0., 0.])
    return c

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
