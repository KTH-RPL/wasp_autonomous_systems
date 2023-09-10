#!/usr/bin/env python

from assignment_3.grid_map import GridMap

import random

from math import ceil


def create_map(resolution: float, width: float, height: float, map_num: int, num_walls: int, robot_radius: float) -> tuple[GridMap, list[tuple[float, float]]]:
    gm = GridMap(resolution, width, height)

    minimum = list(gm.min_coord())
    maximum = list(gm.max_coord())
    minimum[0] += gm.resolution / 2
    minimum[1] += gm.resolution / 2
    maximum[0] -= gm.resolution / 2
    maximum[1] -= gm.resolution / 2

    size = (maximum[0] - minimum[0], maximum[1] - minimum[1])

    robot_size = (ceil(2 * robot_radius / resolution) + 1) * resolution

    environments = [[],
                    [((minimum[0] + size[0] / 2 - resolution / 2, minimum[1]),
                      (minimum[0] + size[0] / 2 - resolution / 2, minimum[1] + size[1] / 2))],
                    [((minimum[0] + size[0] / 2 - resolution / 2, minimum[1]),
                      (minimum[0] + size[0] / 2 - resolution / 2, minimum[1] + size[1] / 2 - (robot_radius + resolution / 2))), 
                     ((minimum[0] + size[0] / 2 - resolution / 2, maximum[1]), 
                      (minimum[0] + size[0] / 2 - resolution / 2, maximum[1] - size[1] / 2 + (robot_radius + resolution / 2)))],
                    [((minimum[0] + size[0] / 2 - resolution / 2, minimum[1]),
                      (minimum[0] + size[0] / 2 - resolution / 2, minimum[1] + size[1] / 2 - robot_size)), 
                     ((minimum[0] + size[0] / 2 - resolution / 2, maximum[1]),
                      (minimum[0] + size[0] / 2 - resolution / 2, maximum[1] - size[1] / 2 + robot_size))],
                    [((minimum[0] + size[0] / 2 - resolution / 2, minimum[1] + 3 * robot_size),
                      (minimum[0] + size[0] / 2 - resolution / 2, minimum[1] + size[1] / 2 - robot_size)),
                     ((minimum[0] + size[0] / 2 - resolution / 2, maximum[1]),
                      (minimum[0] + size[0] / 2 - resolution / 2, maximum[1] - size[1] / 2 + robot_size))]
                    ]

    # Have border for all maps
    walls = [(minimum, (minimum[0], maximum[1])), ((minimum[0], maximum[1]), maximum),
             (maximum, (maximum[0], minimum[1])), ((maximum[0], minimum[1]), minimum)]

    if map_num < len(environments):
        print(f'Creating map #{map_num}, with resolution {resolution}')
        walls.extend(environments[map_num])
    else:
        print(
            f'Creating random map with {num_walls} walls and resolution {resolution}')
        random.seed(map_num)
        for _ in range(num_walls):
            x1 = random.uniform(minimum[0], maximum[0])
            y1 = random.uniform(minimum[1], maximum[1])
            x2 = random.uniform(minimum[0], maximum[0])
            y2 = random.uniform(minimum[1], maximum[1])
            walls.append(((x1, y1), (x2, y2)))

    cells = []
    for wall in walls:
        cells.extend(gm.cells_along_line_segment(*wall[0], *wall[1]))
    for cell in cells:
        gm[cell] = -1

    return gm, walls
