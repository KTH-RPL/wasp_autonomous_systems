#!/usr/bin/env python

from assignment_3.grid_map import GridMap

import itertools


def inflate_map(gm: GridMap, cells_inflate: int):
    for x, y in itertools.product(range(gm.width), range(gm.height)):
        if -1 == gm[x, y]:
            for i, j in itertools.product(range(max(0, x - cells_inflate), min(gm.width, x + cells_inflate + 1)), range(max(0, y - cells_inflate), min(gm.height, y + cells_inflate + 1))):
                if -1 != gm[i, j]:
                    gm[i, j] = 200
