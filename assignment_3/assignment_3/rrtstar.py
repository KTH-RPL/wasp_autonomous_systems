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
        return (target_x, target_y)

    d_x *= max_edge_length / dist
    d_y *= max_edge_length / dist

    return (x + d_x, y + d_y)


def neighbors(vertices: kdtree.KDNode, point: tuple[float, float], radius: float) -> list[tuple[float, float]]:
    neighbors = []
    for neighbor in vertices.search_nn_dist(point, radius):
        neighbors.append(neighbor)
    return neighbors

# Rad = r
# G(V,E) //Graph containing edges and vertices
# For itr in range(0…n)
#     Xnew = RandomPosition()
#     If Obstacle(Xnew) == True, try again
#     Xnearest = Nearest(G(V,E),Xnew)
#     Cost(Xnew) = Distance(Xnew,Xnearest)
#     Xbest,Xneighbors = findNeighbors(G(V,E),Xnew,Rad)
#     Link = Chain(Xnew,Xbest)
#     For x’ in Xneighbors
#         If Cost(Xnew) + Distance(Xnew,x’) < Cost(x’)
#             Cost(x’) = Cost(Xnew)+Distance(Xnew,x’)
#             Parent(x’) = Xnew
#             G += {Xnew,x’}
#     G += Link
# Return G


def rrtstar(gm: GridMap, x1: float, y1: float, x2: float, y2: float, iterations: int = 10000, max_edge_length: float = 2.0, early_stop: bool = False) -> tuple[list[tuple[float, float]], set[tuple[tuple[float, float], tuple[float, float]]]]:
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

    if early_stop:
        # Check if we can reach goal directly from start
        if math.hypot(goal[0] - start[0], goal[1] - start[1]) < max_edge_length and gm.collision_free(*start, *goal):
            return [start, goal], [(start, goal)]

    min_coord = gm.min_coord()
    max_coord = gm.max_coord()

    vertices = kdtree.create([start])

    came_from = {}
    distance = {start: 0}

    edges = set()

    for _ in range(iterations):

        rand = random_coord(*min_coord, *max_coord)
        near = nearest(vertices, rand)
        new = move_closer(*near, *rand, max_edge_length)

        if gm.collision_free(*near, *new):
            cost = distance[near] + \
                math.hypot(new[0] - near[0], new[1] - near[1])
            distance[new] = cost

            # Rewire
            for neighbor in neighbors(vertices, new, max_edge_length):
                if near == neighbor:
                    continue

                new_cost = cost + \
                    math.hypot(neighbor[0] - new[0], neighbor[1] - new[1])

                if new_cost < distance[neighbor]:
                    edges.remove((came_from[neighbor], neighbor))
                    distance[neighbor] = new_cost
                    came_from[neighbor] = new
                    edges.add((new, neighbor))

            came_from[new] = near
            vertices.add(new)
            edges.add((near, new))

            if early_stop:
                # Check if we can connect to goal
                if math.hypot(goal[0] - new[0], goal[1] - new[1]) < max_edge_length and gm.collision_free(*new, *goal):
                    came_from[goal] = new
                    edges.add((new, goal))
                    return reconstruct_path(came_from, goal), edges

    near = nearest(vertices, goal)

    # Check if we can connect to goal
    if math.hypot(goal[0] - near[0], goal[1] - near[1]) < max_edge_length and gm.collision_free(*near, *goal):
        came_from[goal] = near
        edges.add((near, goal))
        return reconstruct_path(came_from, goal), edges

    return [], edges
