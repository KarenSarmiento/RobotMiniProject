from laser import Laser
from pykalman import KalmanFilter, UnscentedKalmanFilter
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from tf import TransformListener
from robot import Robot
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import rospy
from geometry_msgs.msg import Twist
import numpy as np
from sklearn.metrics import pairwise_distances
import os
import sys
from scipy.spatial import distance

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
        super(Leader, self).__init__(name, rate_limiter=rate_limiter,
                                     laser_range=[np.pi/3.5, np.pi/3.5], laser_dist=2)
        self._epsilon = 0.1
        self.leg_pub = rospy.Publisher(
            '/' + name + '/legs', Marker, queue_size=5)
        self.centroid_pub = rospy.Publisher(
            '/' + name + '/centroids', Marker, queue_size=5)
        self.v_pub = rospy.Publisher('/'+name+'/vel', Marker, queue_size=5)
        self.follow = None
        self._tf = TransformListener()
        self.last_vel = np.array([0., 0.])
        self.all_around_laser = Laser(name=name, laser_range=[np.pi, np.pi])
        self.last_legs = None

        def f(state, noise):
            pass

        def g(state, noise):
            pass

        self.ukf = UnscentedKalmanFilter(f, g)
        self.kf = None

    def get_centroids_from_all_around_laser(self):
        return self.all_around_laser.centroids

    def update_velocities(self):
        if not self.laser.ready:
            return

        # follow is relative to robot frame
        follow = self.find_legs()
        print("legs at ", follow)   
        # if self.last_legs is not None:
        #     # predict + update with kalman
        #     follow = self.kf.smooth(follow)
        #     self.kf = self.kf.em(follow)
        # else:
        #     self.kf = KalmanFilter(initial_state_mean=0, n_dim_obs=2)
        #     self.kf = self.kf.em(follow)
        # 20cm away from object is good enough

        goal_reached = np.linalg.norm(follow) < .4
        if goal_reached or not np.isfinite(np.linalg.norm(follow)):
            if goal_reached:
                print("Goal reached")
            else:
                print("nans: ", follow)
            self.publisher.publish(stop_msg)
            self.rate_limiter.sleep()
            return

        # TODO: PID here?
        v = cap(0.1*follow, SPEED)
        print("leader v", v)
        self.publish_v(v, self.v_pub)
        # relative feedback linearization
        u, w = self.feedback_linearized(
            self.slam.pose, v, epsilon=self.epsilon)
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        self.last_vel = np.array([u, w])
        self.pose_history.append(self.slam.pose)
        self._update_pose_history()
        self.publisher.publish(vel_msg)
        self.last_legs = follow

    def feedback_linearized(self, pose, v, epsilon):
        u = v[X]
        w = v[Y] / epsilon
        return u, w*0.9

    def publish_v(self, v, pub):
        m = Marker()
        m.action = Marker.ADD
        m.header.frame_id = '/'+self.name+'/base_link'
        m.header.stamp = rospy.Time.now()
        m.ns = 'points_arrows'
        m.type = Marker.ARROW
        m.color.r = 0.2
        m.color.g = 0.5
        m.color.b = 1.0
        m.color.a = 0.3
        m.scale = Vector3(0.01, 0.02, 0.05)
        p = Point()
        p.x = 0
        p.y = 0
        p1 = Point()
        p1.x = v[X]
        p1.y = v[Y]
        m.points = [p, p1]
        pub.publish(m)
        self.rate_limiter.sleep()

    def find_legs(self):

        centroids = self.laser.centroids
        self.publish_markers(centroids, self.centroid_pub)

        if len(centroids) == 0:
            c = np.array([0., 0.])
            if self.last_legs is not None:
                c = self.last_legs

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
            # if self.last_legs is not None:
            #     possible_pairs = []
            #     possible_centroids = []
            #     for x in centroids:
            #         for y in centroids:
            #             print(x, y)
            #             if 0.1 < np.linalg.norm(y-x) < 0.4:
            #                 possible_pairs.append(np.array([x, y]))
            #                 possible_centroids.append((x+y)/2)
            #     if len(possible_centroids) == 0:
            #         return np.array([0., 0.])
            #     closest_index = distance.cdist(
            #         np.array([self.last_legs]), np.array(possible_centroids)).argmin()
            #     next_centroid = possible_centroids[closest_index]
            #     c = next_centroid

        self.follow = c
        self.abs_leg_pos = c + self.slam.pose[:-1]
        return c

    def publish_leg(self):
        self.publish_markers(
            np.array([self.follow]), self.leg_pub, color_b=1, color_r=0)

    @property
    def epsilon(self):
        return self._epsilon


def cap(v, max_speed):
    n = np.linalg.norm(v)
    if n > max_speed:
        return v / n * max_speed
    return v
