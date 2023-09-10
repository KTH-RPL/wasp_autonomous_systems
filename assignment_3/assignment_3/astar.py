#!/usr/bin/env python

import heapq as hq
from assignment_3.grid_map import GridMap
from assignment_3.utility import reconstruct_path, cell_to_coord_path

import math


def cost(x1: int, y1: int, x2: int, y2: int, eight_connectivity: bool) -> float:

    # TODO: Fill in

    cost = 0.0  # TODO: Fill in

    return cost


def heuristic(x1: int, y1: int, x2: int, y2: int, eight_connectivity: bool) -> float:

    # TODO: Fill in

    h = 0.0  # TODO: Fill in

    return h


def neighbors(gm: GridMap, x: int, y: int, eight_connectivity: bool) -> list[tuple[int, int]]:
    neighbors = []

    # TODO: Fill in

    if gm.inside(x - 1, y) and gm.free(x - 1, y):
        neighbors.append((x - 1, y))
    if gm.inside(x, y - 1) and gm.free(x, y - 1):
        neighbors.append((x, y - 1))
    if gm.inside(x + 1, y) and gm.free(x + 1, y):
        neighbors.append((x + 1, y))
    if gm.inside(x, y + 1) and gm.free(x, y + 1):
        neighbors.append((x, y + 1))

    if eight_connectivity:
        # TODO: Fill in
        pass

    return neighbors


def astar(gm: GridMap, x1: float, y1: float, x2: float, y2: float, eight_connectivity: bool) -> tuple[list[tuple[float, float]], set[tuple[int, int]]]:
    start = gm.cell(x1, y1)
    goal = gm.cell(x2, y2)

    # Check if start and goal are inside the map bounds
    if not gm.inside(*start) or not gm.inside(*goal):
        print('Start and/or goal are outside map bounds')
        return [], set()

    # Check if start and goal are in free space
    if not gm.free(*start) or not gm.free(*goal):
        print('Start and/or goal are inside an obstacle')
        return [], set()

    came_from = {}
    distance = {start: 0}
    visited = set()
    to_be_visited = [(heuristic(*start, *goal, eight_connectivity), 0, start)]
    count = 1

    while to_be_visited:
        current = hq.heappop(to_be_visited)[2]
        visited.add(current)

        if current == goal:
            path = reconstruct_path(came_from, goal)
            return cell_to_coord_path(gm, path), visited

        dist = distance[current]

        for neighbor in neighbors(gm, *current, eight_connectivity):
            neighbor_dist = dist + cost(*current, *neighbor, eight_connectivity)
            if neighbor_dist < distance.get(neighbor, float('inf')):
                came_from[neighbor] = current
                distance[neighbor] = neighbor_dist
                neighbor_min_dist_to_goal = neighbor_dist + \
                    heuristic(*neighbor, *goal, eight_connectivity)
                hq.heappush(
                    to_be_visited, (neighbor_min_dist_to_goal, -count, neighbor))
                count += 1

    return [], visited
