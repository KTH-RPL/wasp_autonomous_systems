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
        """Returns the value of the cell at `[x, y]`, (Free: 0, Occupied: other).

        Does _not_ perform any bounds checks! Use `inside(x, y)` for that.
        """
        x, y = cell
        return self._data[y][x]

    def __setitem__(self, cell: tuple[int, int], value: int):
        """Sets the value of the cell at `[x, y]`, (Free: 0, Occupied: other).

        Does _not_ perform any bounds checks! Use `inside(x, y)` for that.
        """
        x, y = cell
        self._data[y][x] = value

    def free(self, x: int, y: int) -> bool:
        """Returns `true` if the cell `[x, y]` is free (i.e., is 0), `false` otherwise."""
        return 0 == self[x, y]

    @property
    def resolution(self):
        """Map resolution [meter/cell]"""
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
        return self.width * self.resolution

    def euclidean_height(self):
        """Map height in meters"""
        return self.height * self.resolution

    @property
    def origin(self):
        """Origin of the map."""
        return self._origin

    def cell(self, x: float, y: float) -> tuple[int, int]:
        """Converts a Euclidean coordinate `(x, y)` to a grid cell."""
        return (floor((x - self._origin[0]) / self.resolution),
                floor((y - self._origin[1]) / self.resolution))

    def coord(self, x: int, y: int) -> tuple[float, float]:
        """Converts the grid cell `[x, y]` to a Euclidean coordinate."""
        return ((x + 0.5) * self.resolution + self._origin[0],
                (y + 0.5) * self.resolution + self._origin[1])

    def min_coord(self) -> tuple[float, float]:
        """Returns the minimum Euclidean coordinate that the grid covers."""
        return self._origin

    def max_coord(self) -> tuple[float, float]:
        """Returns the maximum Euclidean coordinate that the grid covers."""
        return (self._origin[0] + self.euclidean_width(), self._origin[1] + self.euclidean_height())

    def inside(self, x: int, y: int) -> bool:
        """Returns `true` if the cell `[x, y]` is inside the grid, `false` otherwise."""
        return 0 <= x < self.width and 0 <= y < self.height

    def euclidean_inside(self, x: float, y: float) -> bool:
        """Returns `true` if the Euclidean coordinate `(x, y)` is inside the grid, `false` otherwise."""
        min_c = self.min_coord()
        max_c = self.max_coord()
        return min_c[0] <= x <= max_c[0] and min_c[1] <= y <= max_c[1]

    def collision_free(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        """Returns `true` if the straight line from `(x1, y1)` (including) to `(x2, y2)` (including) is collision free, `false` otherwise."""
        if not self.euclidean_inside(x1, y1) or not self.euclidean_inside(x2, y2):
            return False

        start = self.cell(x1, y1)
        goal = self.cell(x2, y2)

        if start == goal:
            return self.free(*start)

        tmp = self.coord(*start)
        border = (tmp[0] - x1, tmp[1] - y1)

        direction = [x2 - x1, y2 - y1]
        dist = hypot(*direction)
        direction[0] /= dist
        direction[1] /= dist
        step = (int(sign(direction[0])), int(sign(direction[1])))

        t_max = [(border[i] + sign(direction[i]) * self.resolution / 2) /
                 direction[i] if 0 != step[i] else float('inf') for i in (0, 1)]

        t_delta = tuple(self.resolution / fabs(direction[i]) if 0 != step[i] else float(
            'inf') for i in (0, 1))

        steps = sum(
            max(0, ceil((dist - t_max[i]) / t_delta[i])) if 0 != step[i] else 0 for i in (0, 1))

        cur = list(start)
        for _ in range(steps):
            dim = 0 if t_max[0] <= t_max[1] else 1
            t_max[dim] += t_delta[dim]
            cur[dim] += step[dim]
            if not self.free(*cur):
                return False

        return True
    
    def cells_along_line_segment(self, x1: float, y1: float, x2: float, y2: float) -> list[tuple[int, int]]:
        """Returns the cells along the straight line from `(x1, y1)` (including) to `(x2, y2)` (including)"""

        start = self.cell(x1, y1)
        goal = self.cell(x2, y2)

        if not self.inside(*start) or not self.inside(*goal):
            return []

        if start == goal:
            return [start]

        tmp = self.coord(*start)
        border = (tmp[0] - x1, tmp[1] - y1)

        direction = [x2 - x1, y2 - y1]
        dist = hypot(*direction)
        direction[0] /= dist
        direction[1] /= dist
        step = (int(sign(direction[0])), int(sign(direction[1])))

        t_max = [(border[i] + sign(direction[i]) * self.resolution / 2) /
                 direction[i] if step[i] else float('inf') for i in (0, 1)]

        t_delta = tuple(self.resolution / fabs(direction[i]) if step[i] else float(
            'inf') for i in (0, 1))

        steps = sum(
            max(0, ceil((dist - t_max[i]) / t_delta[i])) if step[i] else 0 for i in (0, 1))

        cells = [start]
        for _ in range(steps):
            dim = 0 if t_max[0] <= t_max[1] else 1
            t_max[dim] += t_delta[dim]
            last = list(cells[-1])
            last[dim] += step[dim]
            cells.append((last[0], last[1]))

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
