#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

import numpy as np
import numpy.typing as npt
from numpy import sign

from math import fabs, ceil, hypot, floor
from collections.abc import Sequence


class GridMap(Node):

    def __init__(self, resolution, width, height):
        self._resolution = resolution
        self._width = ceil(width / resolution)
        self._height = ceil(height / resolution)
        self._origin = (-width / 2, -height / 2)
        self._data = np.full((self._height, self._width), 0, dtype=np.int8)

    def __getitem__(self, cell: tuple[int, int]) -> int:
        """Return the value of the cell at (x, y), (Free: 0, Occupied: other).

        Does _not_ perform any bounds checking! Use 'inside(x, y)' for that.
        """
        x, y = cell
        return self._data[y][x]

    def __setitem__(self, cell: tuple[int, int], value: int):
        """Set the value of the cell at (x, y), (Free: 0, Occupied: other).

        Does _not_ perform any bounds checking! Use 'inside(x, y)' for that.
        """
        x, y = cell
        self._data[y][x] = value

    def free(self, x: int, y: int) -> bool:
        """Returns whether the cell [x, y] is free (i.e., is 0)"""
        return 0 == self[x, y]

    @property
    def resolution(self):
        """Map resolution [m/cell]"""
        return self._resolution

    @property
    def width(self):
        """Map width [cells]"""
        return self._width

    @property
    def height(self):
        """Map height [cells]"""
        return self._height

    def euclidean_width(self):
        """Map width in meters"""
        return self.width / self.resolution

    def euclidean_height(self):
        """Map height in meters"""
        return self.height / self.resolution

    @property
    def origin(self):
        return self._origin

    def cell(self, x: float, y: float) -> tuple[int, int]:
        """Converts an Euclidean coordinate to a grid cell."""
        return (floor((x - self._origin[0]) // self.resolution),
                floor((y - self._origin[1]) // self.resolution))

    def coord(self, x: int, y: int) -> tuple[float, float]:
        """Converts a grid cell to en Euclidean coordinate.

        The center of the cell is returned.
        """
        return (x * self.resolution + self._origin[0] + self.resolution / 2,
                y * self.resolution + self._origin[1] + self.resolution / 2)

    def min_coord(self) -> tuple[float, float]:
        """Minimum Euclidean coordinate that the grid map covers"""
        t = self.coord(0, 0)
        return (t[0] - self.resolution / 2, t[1] - self.resolution / 2)

    def max_coord(self) -> tuple[float, float]:
        """Maximum Euclidean coordinate that the grid map covers"""
        t = self.coord(self.width - 1, self.height - 1)
        eps = 0.0001
        return (t[0] - eps + self.resolution / 2, t[1] - eps + self.resolution / 2)

    def inside(self, x: int, y: int) -> bool:
        """Return whether a cell is inside the grid."""
        return 0 <= x < self.width and 0 <= y < self.height

    def collision_free(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        """"""

        return self.inside(*self.cell(x1, y1)) and self.inside(*self.cell(x2, y2)) and all(self.free(*cell) for cell in self.cells_along_line_segment(x1, y1, x2, y2))

    def cells_along_line_segment(self, x1: float, y1: float, x2: float, y2: float) -> list[tuple[int, int]]:
        """"""

        start = self.cell(x1, y1)
        goal = self.cell(x2, y2)

        if start == goal:
            return [start] if self.inside(*start) else []

        direction = [x2 - x1, y2 - y1]

        step = (int(sign(direction[0])), int(sign(direction[1])))
        delta = tuple(step[i] * self.resolution / direction[i] if step[i] else float(
            'inf') for i in (0, 1))

        p = (x1, y1)

        t_max = [float('inf'), float('inf')]
        for i in (0, 1):
            if 0 < step[i]:
                t_max[i] = delta[i] * (1 - p[i] + floor(p[i]))
            elif 0 > step[i]:
                t_max[i] = delta[i] * (p[i] - floor(p[i]))

        cells = [start] if self.inside(*start) else []
        cur = list(start)
        while True:
            dim = 0 if t_max[0] <= t_max[1] else 1
            cur[dim] += step[dim]
            t_max[dim] += delta[dim]
            if self.inside(*cur):
                cells.append((cur[0], cur[1]))

            if 1 < t_max[0] and 1 < t_max[1]:
                break

        return cells

    def to_ros(self, frame_id: str = '', stamp=None) -> OccupancyGrid:
        og = OccupancyGrid()
        og.header.frame_id = frame_id
        if stamp:
            og.header.stamp = stamp

        og.info.resolution = self.resolution
        og.info.width = self.width
        og.info.height = self.height
        og.info.origin.position.x = self._origin[0]
        og.info.origin.position.y = self._origin[1]

        og.data = self._data.flatten().tolist()

        return og


def optimize_path(map: GridMap, path: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    # if not path:
    #     return []

    # forward = [0]
    # i = 0
    # while len(path) - 1 != i:
    #     for j in range(len(path) - 1, i, -1):
    #         if map.collision_free(*path[forward[-1]], *path[j]):
    #             forward.append(j)
    #             i = j
    #             break

    # backward = [len(path) - 1]
    # i = len(path) - 1
    # while i:
    #     for j in range(i):
    #         if map.collision_free(*path[backward[-1]], *path[j]):
    #             backward.append(j)
    #             i = j
    #             break

    # backward.reverse()

    # return [path[x] for x in backward]

    if not path:
        return []

    op = [path[0]]
    i = 0
    while len(path) - 1 != i:
        for j in range(len(path) - 1, i, -1):
            if map.collision_free(*op[-1], *path[j]):
                op.append(path[j])
                i = j
                break
    return op
