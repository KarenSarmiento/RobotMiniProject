#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import os
import rospy
import sys

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Occupancy grid.
from nav_msgs.msg import OccupancyGrid
# Position.
from tf import TransformListener
# Goal.
from geometry_msgs.msg import PoseStamped
# Path.
from nav_msgs.msg import Path
# For pose information.
from tf.transformations import euler_from_quaternion
# For converting laser to point cloud
import laser_geometry.laser_geometry as lg
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
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
    print(i, v, d)
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
                print(self._pose)
                print(self._occupancy_grid)
            except Exception as e:
                print(e)
        else:
            print('Unable to find:', self._tf.frameExists(
                a), self._tf.frameExists(b))
        pass

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
        pc2_msg = self._lp.projectLaser(msg)
        self._point_gen = pc2.read_points(pc2_msg)
        
        self._angles = np.arange(
            msg.angle_min, msg.angle_max, msg.angle_increment)
        self._measurements = np.array(msg.ranges)
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
        return self._point_gen


def get_follow_position(pose, laser):
    cone = np.where((laser.angles > (7*np.pi/4)) | (laser.angles < (np.pi/4)))
    cone_angles = laser.angles[cone]
    cone_measures = laser.measurements[cone]
    close = np.where(cone_measures < 2.5)
    close_angles = cone_angles[close]
    closest = np.argmin(cone_measures)
    closest_dist = cone_measures[closest]
    closest_angle = cone_angles[closest]

    x = pose[X] + closest_dist * np.cos(closest_angle)
    y = pose[Y] + closest_dist * np.sin(closest_angle)
    position = np.array([x, y])
    print(position)
    return position


def run(args):
    rospy.init_node('rrt_navigation')

    # Update control every 100 ms.
    rate_limiter = rospy.Rate(100)
    publisher = rospy.Publisher('/'+ROBOT_NAME+'/cmd_vel', Twist, queue_size=5)
    path_publisher = rospy.Publisher(
        '/'+ROBOT_NAME+'/path', Path, queue_size=1)
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

    while not rospy.is_shutdown():
        slam.update()
        current_time = rospy.Time.now().to_sec()
        # Make sure all measurements are ready.
        # Get map and current position through SLAM:
        # > roslaunch exercises slam.launch
        print(goal.ready, slam.ready)
        if not goal.ready or not slam.ready or not laser.ready:
            rate_limiter.sleep()
            continue

        goal_reached = np.linalg.norm(slam.pose[:2] - goal.position) < .2
        if goal_reached:
            print("Goal Reached")
            publisher.publish(stop_msg)
            rate_limiter.sleep()
            continue

        # Follow path using feedback linearization.
        position = np.array([
            slam.pose[X] + EPSILON * np.cos(slam.pose[YAW]),
            slam.pose[Y] + EPSILON * np.sin(slam.pose[YAW])], dtype=np.float32)
        print(position)
        v = get_velocity(position, np.array(current_path, dtype=np.float32))
        u, w = feedback_linearized(slam.pose, v, epsilon=EPSILON)
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        publisher.publish(vel_msg)

        # Update plan every 1s.
        time_since = current_time - previous_time
        if current_path and time_since < 2.:
            rate_limiter.sleep()
            continue
        previous_time = current_time

        follow_position = get_follow_position(slam.pose, laser)
        # Run RRT.
        start_node, final_node = rrt.rrt(
            slam.pose, follow_position, slam.occupancy_grid)
        current_path = get_path(final_node)
        if not current_path:
            print('Unable to reach goal position:', follow_position)

        # Publish path to RViz.
        path_msg = Path()
        path_msg.header.seq = frame_id
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'map'
        for u in current_path:
            pose_msg = PoseStamped()
            pose_msg.header.seq = frame_id
            pose_msg.header.stamp = path_msg.header.stamp
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = u[X]
            pose_msg.pose.position.y = u[Y]
            path_msg.poses.append(pose_msg)
        path_publisher.publish(path_msg)

        rate_limiter.sleep()
        frame_id += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs RRT navigation')
    args, unknown = parser.parse_known_args()
    try:
        run(args)
    except rospy.ROSInterruptException:
        pass
