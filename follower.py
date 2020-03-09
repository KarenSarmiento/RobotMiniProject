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
    def __init__(self, name, rate_limiter, desired_rel_pos, des_d, des_psi, leader, laser_range=[np.pi/4, np.pi/4]):
        super(Follower, self).__init__(
            name, rate_limiter=rate_limiter, map_frame=name+"/occupancy_grid", laser_range=laser_range)
        self._desired_rel_pos = desired_rel_pos
        self._epsilon = 0.1
        self.last_leader_pos = None
        self.centroid_pub = rospy.Publisher(
            '/' + name + '/centroids', Marker, queue_size=5)
        self.last_pose = None
        self.des_d = des_d
        self.des_psi = des_psi
        self.leader = leader
        self.offset = None

    def formation_velocity(self):
        print("---", self.name, "---")
        self.slam.update()
        # Return if not ready.
        if not self.laser.ready:  # or not self.slam.ready:
            self.rate_limiter.sleep()
            return
        # pose of leader relative to self
        leader_pose = self.get_leader_pose()

        print("leader pose from {} is {}".format(self.name, leader_pose))
        # u, w = self.get_controls_p(leader_pose)
        u, w = self.get_controls_formula(leader_pose)

        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        self.publisher.publish(vel_msg)
        self.pose_history.append(self.slam.pose)

        self._update_pose_history()

    def get_controls_formula(self, leader_pose):

        l_ij = np.linalg.norm(leader_pose[X:Y+1])
        b_ij = leader_pose[YAW]
        psi_ij = np.pi + np.arctan2(leader_pose[Y], leader_pose[X]) - b_ij
        gamma_ij = b_ij + psi_ij

        G = np.array([[np.cos(gamma_ij),       0.05*np.sin(gamma_ij)],
                      [-np.sin(gamma_ij)/l_ij,  (0.05*np.cos(gamma_ij))/l_ij]])
        F = np.array([[-np.cos(psi_ij),        0],
                      [np.sin(psi_ij)/l_ij,   -1]])
        z_ij = np.array([[l_ij, psi_ij]]).transpose()
        zd_ij = np.array([[self.des_d, self.des_psi]]).transpose()
        k = np.array([[1.5], [0.9]])

        # print("last_vel", self.leader.last_vel)
        u_j = np.dot(np.linalg.inv(G), (k*(zd_ij-z_ij) -
                                        np.dot(F, np.array([self.leader.last_vel]).transpose())))
        # # print(u,w)
        u = u_j[0][0] * 0.1
        w = u_j[1][0] * 0.01
        # # Publish control inputs[]
        # print("F", F)
        # print("G", G)
        # print("uw:",u,w)
        # print("leader at", leader_pose)

        return u, w

    def get_controls_p(self, leader_pose):

        rotated_desired_rel_pos = _rotate(
            self._desired_rel_pos[:-1], leader_pose[YAW])

        rotated_desired_rel_pos = leader_pose[X:Y+1] + rotated_desired_rel_pos
        print("desired position for {} is {}".format(self.name, rotated_desired_rel_pos))
        if np.linalg.norm(rotated_desired_rel_pos) < 0.3:
            print("follower goal reached")
            self.publisher.publish(stop_msg)
            return 0, 0

        # P control
        error = rotated_desired_rel_pos
        v = cap(0.6 * error, SPEED)

        # Translate velocity to control inputs for non-holonomic robot.
        u, w = self.feedback_linearized_rel(v)
        return u, w*1.2

        goal_pos = leader_pose[X:Y+1] + self._desired_rel_pos[X:Y+1]
        goal_yaw = leader_pose[YAW]
        u, w = self.p_control(goal_pos, goal_yaw)

    def p_control(self, goal_pos, goal_yaw):
        err_u = np.linalg.norm(goal_pos)
        err_w = goal_yaw

        u = err_u*0.2
        w = err_w*0.2
        if np.linalg.norm(goal_pos) < 0.5:
            u = 0
        return u, w

    def feedback_linearized_rel(self, v):
        # relative feedback linearization
        u = v[X]
        w = v[Y] / self._epsilon
        return u, w


    def get_leader_position(self):
        # return estimate of relative position of leader through clustering
        cs = self.laser.centroids
        if cs is None or len(cs) == 0:
            return np.array([0., 0.])
        print("possible leaders: {}".format(cs))
        if self.last_leader_pos is not None:
            closest_index = distance.cdist([self.last_leader_pos], cs).argmin()
            rel_leader_pos = cs[closest_index]
        else:
            # Get closest cluster to itself if there is no last leader position (start)
            closest_index = distance.cdist([-self._desired_rel_pos[X:Y+1]], cs).argmin()
            rel_leader_pos = cs[closest_index]



        self.last_leader_pos = rel_leader_pos
        self.publish_markers(
            np.array([rel_leader_pos]), self.centroid_pub, color_r=0.5)
        print("chosen leader is {}".format(rel_leader_pos))
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
        if self.offset is not None:
            # leader_movement = -last_lead_pos + movement[X:Y+1] + new_lead_pos
            # pose[YAW] = np.arctan2(leader_movement[Y], leader_movement[X])
            print(self.leader.slam.pose[YAW], self.slam.pose[YAW])
            pose[YAW] = (self.leader.slam.pose[YAW] - self.slam.pose[YAW]) + self.offset
        elif self.leader.slam.ready and self.slam.ready:
            #Assume all robots in same direction as start, same (yaw)
            self.offset = np.abs(self.leader.slam.pose[YAW] -  self.slam.pose[YAW])
            print("setting offset", self.offset)
            pose[YAW] = 0.
        
        pose = np.nan_to_num(pose)
        print("relative yaw is {}".format(pose[YAW]))
        return pose


def _rotate(vector, angle, axis=[0.0, 0.0, 1.0]):
    return Quaternion(axis=axis, angle=angle).rotate(np.append(vector, [0.0]))[:2]


def cap(v, max_speed):
    n = np.linalg.norm(v)
    if n > max_speed:
        return v / n * max_speed
    return v
