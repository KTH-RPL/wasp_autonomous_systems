#!/usr/bin/env python

import kdtree
from assignment_3.grid_map import GridMap
from assignment_3.utility import reconstruct_path

import random
import math


def random_coord(x_min: float, y_min: float, x_max: float, y_max: float) -> tuple[float, float]:
    rand_x = random.uniform(x_min, x_max)
    rand_y = random.uniform(y_min, y_max)

    return (rand_x, rand_y)


def nearest(vertices: kdtree.KDNode, point: tuple[float, float]) -> tuple[float, float]:
    return vertices.search_nn(point)[0].data


def move_closer(x: float, y: float, target_x: float, target_y: float, max_edge_length: float) -> tuple[float, float]:
    d_x = target_x - x
    d_y = target_y - y
    dist = math.hypot(d_x, d_y)

    if dist < max_edge_length:
        return (closer_x, closer_y)

    d_x /= dist
    d_y /= dist

    closer_x = d_x * max_edge_length
    closer_y = d_y * max_edge_length

    return (closer_x, closer_y)


def rrt(gm: GridMap, x1: float, y1: float, x2: float, y2: float, iterations: int = 10000, max_edge_length: float = 2.0) -> tuple[list[tuple[float, float]], list[tuple[tuple[float, float], tuple[float, float]]]]:
    start = (x1, y1)
    goal = (x2, y2)

    start_cell = gm.cell(*start)
    goal_cell = gm.cell(*goal)

    # Check if start and goal are inside the map bounds
    if not gm.inside(*start_cell) or not gm.inside(*goal_cell):
        print('Start and/or goal are outside map bounds')
        return [], []

    # Check if start and goal are in free space
    if not gm.free(*start_cell) or not gm.free(*goal_cell):
        print('Start and/or goal are inside an obstacle')
        return [], []

    # # Check if we can reach goal directly from start
    # if gm.collision_free(*start, *goal):
    #     return [start, goal], [(start, goal)]

    min_coord = gm.min_coord()
    max_coord = gm.max_coord()

    vertices = kdtree.create([start])

    came_from = {}

    edges = []

    for _ in range(iterations):

        rand = random_coord(*min_coord, *max_coord)
        near = nearest(vertices, rand)
        new = move_closer(*near, *rand, max_edge_length)

        if gm.collision_free(*near, *new):
            came_from[new] = near
            vertices.add(new)
            edges.append((near, (new[0], new[1])))

            # # Check if we can connect to goal
            # if gm.collision_free(*new, *goal):
            #     came_from[goal] = new
            #     edges.append(((new[0], new[1]), goal))
            #     return reconstruct_path(came_from, goal), edges

    near = nearest(vertices, goal)

    # Check if we can connect to goal
    if gm.collision_free(*near, *goal):
        came_from[goal] = near
        edges.append(((near[0], near[1]), goal))
        return reconstruct_path(came_from, goal), edges

    return [], edges
