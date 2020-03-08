from visualization_msgs.msg import Marker
import rospy
from pyquaternion import Quaternion
import numpy as np
from geometry_msgs.msg import Twist
from robot import Robot
from scipy.spatial import distance
import os  # noqa
import sys  # noqa
directory = os.path.join(os.path.dirname(  # noqa
    os.path.realpath(__file__)), '.')  # noqa
sys.path.insert(0, directory)  # noqa


X = 0
Y = 1
YAW = 2


class Follower(Robot):
    def __init__(self, name, rate_limiter, desired_rel_pos):
        super(Follower, self).__init__(name, rate_limiter=rate_limiter)
        self._desired_rel_pos = desired_rel_pos
        self._epsilon = 0.1
        self.last_leader_pos = None
        self.centroid_pub = rospy.Publisher(
            '/' + name + '/centroids', Marker, queue_size=5)

    def formation_velocity(self):
        # Return if not ready.
        if not self.laser.ready or not self.slam.ready:
            self.rate_limiter.sleep()
            return

        leader_pose = self.get_leader_pose()

        # Define position of holonomic point to be directly in front of the robot.
        hol_point_pos = np.array([
            self.slam.pose[X] + self._epsilon * np.cos(self.slam.pose[YAW]),
            self.slam.pose[Y] + self._epsilon * np.sin(self.slam.pose[YAW])
        ])

        # Get velocity for holonomic point.
        rotated_desired_rel_pos = _rotate(
            self._desired_rel_pos[:-1], leader_pose[YAW])
        v = self.get_velocity_using_p(
            hol_point_pos, leader_pose[:-1] + rotated_desired_rel_pos)

        # Translate velocity to control inputs for non-holeonomic robot.
        u, w = self.feedback_linearized(self.slam.pose, v, self._epsilon)

        # Publish control inputs
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        self.publisher.publish(vel_msg)
        self.pose_history.append(self.slam.pose)

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

    def get_leader_position(self):
        # return estimate of relative position of leader through clustering
        cs = self.laser.centroids
        if self.last_leader_pos:
            closest_index = distance.cdist([self.last_leader_pos], cs).argmin()
            rel_leader_pos = cs[closest_index]
            self.last_leader_pos = rel_leader_pos
            return rel_leader_pos
        else:
            return cs[0]

    def get_leader_pose(self):
        pose = np.array([0, 0, 0], dtype='float32')
        last_pos = self.last_leader_pos
        new_pos = self.get_leader_position()
        pose[X] = new_pos[X]
        pose[Y] = new_pos[Y]
        vec = new_pos - last_pos
        pose[YAW] = np.arctan2(vec[Y], vec[X])
        return pose


def _rotate(vector, angle, axis=[0.0, 0.0, 1.0]):
    return Quaternion(axis=axis, angle=angle).rotate(np.append(vector, [0.0]))[:2]
