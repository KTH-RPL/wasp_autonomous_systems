#!/usr/bin/env python

from collections import deque
from assignment_3.grid_map import GridMap
from assignment_3.utility import reconstruct_path, cell_to_coord_path


def neighbors(gm: GridMap, x: int, y: int, eight_connectivity: bool) -> list[tuple[int, int]]:
    neighbors = []

    if gm.inside(x - 1, y) and gm.free(x - 1, y):
        neighbors.append((x - 1, y))
    if gm.inside(x, y - 1) and gm.free(x, y - 1):
        neighbors.append((x, y - 1))
    if gm.inside(x + 1, y) and gm.free(x + 1, y):
        neighbors.append((x + 1, y))
    if gm.inside(x, y + 1) and gm.free(x, y + 1):
        neighbors.append((x, y + 1))

    if eight_connectivity:
        # Not applicable
        pass

    return neighbors


def bf(gm: GridMap, x1: float, y1: float, x2: float, y2: float, eight_connectivity: bool) -> tuple[list[tuple[float, float]], set[tuple[int, int]]]:
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
    seen = set([start])
    visited = set()
    to_be_visited = deque([start])

    while to_be_visited:
        current = to_be_visited.popleft()
        visited.add(current)

        if current == goal:
            path = reconstruct_path(came_from, goal)
            return cell_to_coord_path(gm, path), visited

        for neighbor in neighbors(gm, *current, eight_connectivity):
            if neighbor not in seen:
                seen.add(neighbor)
                came_from[neighbor] = current
                to_be_visited.append(neighbor)

    return [], visited
