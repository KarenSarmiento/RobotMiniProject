from sklearn.metrics import pairwise_distances
from formation import Robot
import numpy as np
from geometry_msgs.msg import Twist

X = 0
Y = 1
YAW = 2
SPEED = 0.2
stop_msg = Twist()
stop_msg.linear.x = 0.
stop_msg.angular.z = 0.


class Leader(Robot):
    def __init__(self, name):
        super(Leader, self).__init__(name)
        self._epsilon = 0.1

    def update_velocities(self, rate_limiter):
        c = self.find_legs()
        # If we are at most 20cm away from the "legs", stop
        if not np.isfinite(np.linalg.norm(c)):
            print(c, " is nan")
            return np.array([0., 0])
        if np.linalg.norm(c) < 0.2:
            print("close enough, stopping")
        print(np.linalg.norm(c), " away from centroid")
        # feedback linearize towards centroids which look like legs

        position = np.array([
            self.slam.pose[X] + self.epsilon * np.cos(self.slam.pose[YAW]),
            self.slam.pose[Y] + self.epsilon * np.sin(self.slam.pose[YAW])], dtype=np.float32)
        # v = get_velocity(position, np.array(current_path, dtype=np.float32))
        # follow is relative to robot frame
        follow = get_follow_position(position, laser)

        # 20cm away from object is good enough
        goal_reached = np.linalg.norm(follow) < .2
        if goal_reached:
            print("Goal Reached")
            self.publisher.publish(stop_msg)
            rate_limiter.sleep()
            continue

        v = cap(0.2*follow, SPEED)
        print("v: ", v)
        u, w = self.feedback_linearized(
            self.slam.pose, v, epsilon=self.epsilon)
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        print(vel_msg)
        self.publisher.publish(vel_msg)

    def find_legs(self):
        c = self.laser.centroids

        centroids = self.laser.centroids
        # publish_points("/centroids", centroids)

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
                return np.array([0., 0.])

        return c

    @property
    def epsilon(self):
        return self._epsilon


def cap(v, max_speed):
    n = np.linalg.norm(v)
    if n > max_speed:
        return v / n * max_speed
    return v
