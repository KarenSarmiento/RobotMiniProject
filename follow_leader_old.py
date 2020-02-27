#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np

X = 0
Y = 1
YAW = 2


# TODO: make robots broadcast their position for testing.
# TODO: Take into consideration YAW
class FollowerController(object):
    def __init__(self, desired_rel_pos, epsilon):
        self._desired_rel_pos = desired_rel_pos
        self._epsilon = epsilon

    def run(self, position, leader_position):
        self._update_rel_pos(position, leader_position)
        self._move_to_desired_rel_pos()

    def _update_rel_pos(self, position, leader_position):
        # Get positions relative to leader.
        position -= leader_position
        leader_position = np.zeros_like(position)
        self._rel_pos = position
    
    def _move_to_desired_rel_pos(self):
        # Define position of holonomic point to be directly in front of the robot.
        hol_point_pos = np.array([
            self._rel_pos[X] + self._epsilon * np.cos(self._rel_pos[YAW]),
            self._rel_pos[Y] + self._epsilon * np.sin(self._rel_pos[YAW])
        ])

        # Get velocity for holonimc point/
        v = get_velocity_using_p(hol_point_pos, self._desired_rel_pos)

        # Translate to control inputs for non-holonomic robot.
        u, w = feedback_linearized(self._rel_pos, v, self._epsilon)
        return u, w


# TODO: Move this inside class when make use if i and d components.
def get_velocity_using_p(position, goal):
  # Stop moving if the goal is reached.
  if np.linalg.norm(position - goal) < .2:
    return np.zeros_like(position)

  # Calculate errors for P controller.
  error = goal - position

  # Return velocity.
  speed_sf = 3.0
  return speed_sf * error 

        
def feedback_linearized(pose, velocity, epsilon):
        """Implement feedback-linearization.
        
        Args:
            pose: Current robot state.
            velocity: Vector [x,y] describing the x and y velocities of the holonomic point.
            epsilon: Distance of the linearised point in front of the robot.

        Returns:
            (u, w) which represents the velocity [m/s] and rotational velocity [rads/s] 
            (counter clockwise) of the robot.
        """
        u = velocity[X] * np.cos(pose[YAW]) + velocity[Y] * np.sin(pose[YAW])
        w = (-velocity[X] * np.sin(pose[YAW]) + velocity[Y] * np.cos(pose[YAW])) / epsilon

        return u, w