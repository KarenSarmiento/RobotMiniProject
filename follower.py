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

stop_msg = Twist()
stop_msg.linear.x = 0.
stop_msg.angular.z = 0.

X = 0
Y = 1
YAW = 2
SPEED = 0.2


class Follower(Robot):
    def __init__(self, name, rate_limiter, desired_rel_pos, des_d, des_psi, leader):
        super(Follower, self).__init__(
            name, rate_limiter=rate_limiter, map_frame=name+"/occupancy_grid")
        self._desired_rel_pos = desired_rel_pos
        self._epsilon = 0.1
        self.last_leader_pos = None
        self.centroid_pub = rospy.Publisher(
            '/' + name + '/centroids', Marker, queue_size=5)
        self.last_pose = None
        self.des_d = des_d
        self.des_psi = des_psi
        self.leader = leader

    def formation_velocity(self):
        self.slam.update()
        # Return if not ready.
        if not self.laser.ready:  # or not self.slam.ready:
            self.rate_limiter.sleep()
            return
        print(self.slam.pose)
        # pose of leader relative to self
        leader_pose = self.get_leader_pose()
        print("leader pose from ", self.name, leader_pose)
        # Define position of holonomic point to be directly in front of the robot.
        hol_point_pos = np.array([
            self.slam.pose[X] + self._epsilon * np.cos(self.slam.pose[YAW]),
            self.slam.pose[Y] + self._epsilon * np.sin(self.slam.pose[YAW])
        ])
        hol_point_pos = np.array([0., 1])
        # Get velocity for holonomic point.
        rotated_desired_rel_pos = _rotate(
            self._desired_rel_pos[:-1], leader_pose[YAW])

        rotated_desired_rel_pos = leader_pose[X:Y+1] + rotated_desired_rel_pos
        # if np.linalg.norm(rotated_desired_rel_pos) < 0.3:
        #     self.publisher.publish(stop_msg)
        #     return

        print("goal relative to ", self.name, rotated_desired_rel_pos)
        v = self.get_velocity_using_p(
            hol_point_pos, leader_pose[:-1] + rotated_desired_rel_pos)
        v = cap(v*0.3, SPEED)
        # Translate velocity to control inputs for non-holeonomic robot.
        u, w = self.feedback_linearized(self.slam.pose, v, self._epsilon)

        l_ij = np.linalg.norm(leader_pose[X:Y+1])
        b_ij = leader_pose[YAW]
        psi_ij = np.pi + np.arctan2(leader_pose[Y], leader_pose[X]) - b_ij
        gamma_ij = b_ij + psi_ij

        print("psi", psi_ij)
        print("beta", b_ij)
        print("l", l_ij)
        G = np.array([[np.cos(gamma_ij),       0.05*np.sin(gamma_ij)],
                      [-np.sin(gamma_ij)/l_ij,  (0.05*np.cos(gamma_ij))/l_ij]])
        F = np.array([[-np.cos(psi_ij),        0],
                      [np.sin(psi_ij)/l_ij,   -1]])
        z_ij = np.array([[l_ij, psi_ij]]).transpose()
        zd_ij = np.array([[self.des_d, self.des_psi]]).transpose()
        k = np.array([[0.5],[0.2]])

        goal_pos = leader_pose[X:Y+1] + self._desired_rel_pos[X:Y+1]
        goal_yaw = leader_pose[YAW]
        u, w = self.p_control(goal_pos, goal_yaw)
        print("goal pos", goal_pos)
        # print("last_vel", self.leader.last_vel)
        # u_j = np.dot(np.linalg.inv(G), (k*(zd_ij-z_ij) -
        #                                 np.dot(F, np.array([self.leader.last_vel]).transpose())))
        # # print(u,w)
        # u = u_j[0][0] * 0.1
        # w = u_j[1][0] * 0.01
        # # Publish control inputs[]
        # print("F", F)
        # print("G", G)
        # print("uw:",u,w)
        # print("leader at", leader_pose)
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        self.publisher.publish(vel_msg)
        self.pose_history.append(self.slam.pose)

        self._update_pose_history()

    def p_control(self, goal_pos, goal_yaw):
        err_u = np.linalg.norm(goal_pos)
        err_w = goal_yaw

        u = err_u*0.2
        w = err_w*0.2
        if np.linalg.norm(goal_pos) < 0.5:
            u = 0
        return u, w

    def feedback_linearized(self, pose, v, epsilon):
        # relative feedback linearization
        u = v[X]
        w = v[Y] / epsilon
        return u, w

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
        if cs is None or len(cs) == 0:
            return np.array([0., 0.])
        print("---", self.name, "---")
        print("possible leaders:", cs)
        if self.last_leader_pos is not None:
            closest_index = distance.cdist([self.last_leader_pos], cs).argmin()
            rel_leader_pos = cs[closest_index]
        else:
            # Get closest cluster to itself if there is no last leader position (start)
            closest_index = distance.cdist([[0., 0.]], cs).argmin()
            rel_leader_pos = cs[closest_index]
        closest_index = distance.cdist([[0., 0.]], cs).argmin()
        rel_leader_pos = cs[closest_index]
        self.last_leader_pos = rel_leader_pos
        self.publish_markers(
            np.array([rel_leader_pos]), self.centroid_pub, color_r=0.5)
        print("chosen leader", rel_leader_pos)
        # if np.linalg.norm(rel_leader_pos) < 0.5:
        #     self.publisher.publish(stop_msg)

        return rel_leader_pos

    def get_leader_pose(self):
        pose = np.array([0, 0, 0], dtype='float32')

        self.new_pose = self.slam.pose
        if self.last_pose is not None:
            movement = np.nan_to_num(self.new_pose - self.last_pose)
        else:
            movement = np.array([0., 0.])
        self.last_pose = self.new_pose

        last_lead_pos = self.last_leader_pos
        new_lead_pos = self.get_leader_position()
        pose[X] = new_lead_pos[X]
        pose[Y] = new_lead_pos[Y]
        if last_lead_pos is not None:
            leader_movement = -last_lead_pos + movement[X:Y+1] + new_lead_pos
            pose[YAW] = np.arctan2(leader_movement[Y], leader_movement[X])
        else:
            pose[YAW] = 0
        return pose


def _rotate(vector, angle, axis=[0.0, 0.0, 1.0]):
    return Quaternion(axis=axis, angle=angle).rotate(np.append(vector, [0.0]))[:2]


def cap(v, max_speed):
    n = np.linalg.norm(v)
    if n > max_speed:
        return v / n * max_speed
    return v
