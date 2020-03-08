import numpy as np
import scipy
import pylab as plt
# Constants used for indexing.
X = 0
Y = 1
YAW = 2

# Constants for occupancy grid.
FREE = 0
UNKNOWN = 1
OCCUPIED = 2

ROBOT_RADIUS = 0.105 / 2.


class OccupancyGrid(object):
    def __init__(self, values, origin, resolution):
        self._original_values = values.copy()
        self._values = values.copy()
        # Inflate obstacles (using a convolution).
        inflated_grid = np.zeros_like(values)
        inflated_grid[values == OCCUPIED] = 1.
        w = 2 * int(ROBOT_RADIUS / resolution) + 1
        inflated_grid = scipy.signal.convolve2d(
            inflated_grid, np.ones((w, w)), mode='same')
        self._values[inflated_grid > 0.] = OCCUPIED
        self._origin = np.array(origin[:2], dtype=np.float32)
        self._origin -= resolution / 2.
        assert origin[YAW] == 0.
        self._resolution = resolution

    @property
    def values(self):
        return self._values

    @property
    def resolution(self):
        return self._resolution

    @property
    def origin(self):
        return self._origin

    def draw(self):
        plt.imshow(self._original_values.T, interpolation='none', origin='lower',
                   extent=[self._origin[X],
                           self._origin[X] +
                           self._values.shape[0] * self._resolution,
                           self._origin[Y],
                           self._origin[Y] + self._values.shape[1] * self._resolution])
        plt.set_cmap('gray_r')

    def get_index(self, position):
        idx = ((position - self._origin) / self._resolution).astype(np.int32)
        if len(idx.shape) == 2:
            idx[:, 0] = np.clip(idx[:, 0], 0, self._values.shape[0] - 1)
            idx[:, 1] = np.clip(idx[:, 1], 0, self._values.shape[1] - 1)
            return (idx[:, 0], idx[:, 1])
        idx[0] = np.clip(idx[0], 0, self._values.shape[0] - 1)
        idx[1] = np.clip(idx[1], 0, self._values.shape[1] - 1)
        return tuple(idx)

    def get_position(self, i, j):
        return np.array([i, j], dtype=np.float32) * self._resolution + self._origin

    def is_occupied(self, position):
        return self._values[self.get_index(position)] == OCCUPIED

    def is_free(self, position):
        return self._values[self.get_index(position)] == FREE
