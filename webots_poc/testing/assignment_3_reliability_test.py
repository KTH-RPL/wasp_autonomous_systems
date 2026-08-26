#!/usr/bin/env python3
"""Repeated-run reliability test for Assignment 3 (path planning).
No simulator involved -- exercises the planner functions directly against
many maps and start/goal pairs, including many repeats of the two
randomized planners (rrt, rrtstar) to catch flakiness. Uses `bf` (a
complete, correct 4-connectivity search) as ground truth for reachability,
and GridMap.collision_free() to validate every returned path is actually
safe, not just non-empty.
"""

import random
import signal
import sys
import time
import traceback

from wasp_as_ass_3.create_map import create_map
from wasp_as_ass_3.breadth_first import bf
from wasp_as_ass_3.astar import astar
from wasp_as_ass_3.dijkstra import dijkstra
from wasp_as_ass_3.rrt import rrt
from wasp_as_ass_3.rrtstar import rrtstar

PER_CALL_TIMEOUT_S = 20
RESOLUTION = 0.25
WIDTH = 20.0
HEIGHT = 20.0
ROBOT_RADIUS = 0.3
MAP_IDS = list(range(10))          # 0-4 fixed hand-designed maps, 5-9 random (seeded by id)
RANDOM_PAIRS_PER_MAP = 6
RRT_REPEATS = 5
RRT_ITERATIONS = 10000
RRT_MAX_EDGE = 0.5


class PlannerTimeout(Exception):
    pass


def _alarm_handler(signum, frame):
    raise PlannerTimeout()


def run_bounded(fn, *args, **kwargs):
    signal.signal(signal.SIGALRM, _alarm_handler)
    signal.alarm(PER_CALL_TIMEOUT_S)
    try:
        t0 = time.time()
        result = fn(*args, **kwargs)
        elapsed = time.time() - t0
        return ('OK', result, elapsed)
    except PlannerTimeout:
        return ('TIMEOUT', None, float(PER_CALL_TIMEOUT_S))
    except Exception:
        return ('CRASH', traceback.format_exc(), 0.0)
    finally:
        signal.alarm(0)


def random_free_point(gm, rng):
    lo = gm.min_coord()
    hi = gm.max_coord()
    for _ in range(300):
        x = rng.uniform(lo[0], hi[0])
        y = rng.uniform(lo[1], hi[1])
        cell = gm.cell(x, y)
        if gm.inside(*cell) and gm.free(*cell):
            return (x, y)
    return None


def validate_path(gm, path, start, goal):
    if not path:
        return True, None
    tol = RESOLUTION * 1.5
    if abs(path[0][0] - start[0]) > tol or abs(path[0][1] - start[1]) > tol:
        return False, f'path start {path[0]} far from requested start {start}'
    if abs(path[-1][0] - goal[0]) > tol or abs(path[-1][1] - goal[1]) > tol:
        return False, f'path end {path[-1]} far from requested goal {goal}'
    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        if not gm.collision_free(x1, y1, x2, y2):
            return False, f'segment {path[i]} -> {path[i + 1]} not collision-free'
    return True, None


def main():
    rng = random.Random(1234)  # for start/goal sampling only, not map generation

    stats = {
        name: {'runs': 0, 'crashes': 0, 'timeouts': 0, 'invalid_paths': 0,
               'found': 0, 'no_path': 0, 'max_time': 0.0, 'total_time': 0.0}
        for name in ('bf', 'astar_4', 'astar_8', 'dijkstra_4', 'dijkstra_8', 'rrt', 'rrtstar')
    }
    bugs = []

    total_pairs = 0
    for map_id in MAP_IDS:
        num_walls = 5 if map_id % 3 == 0 else (8 if map_id % 3 == 1 else 12)
        gm, walls = create_map(RESOLUTION, WIDTH, HEIGHT, map_id, num_walls, ROBOT_RADIUS, [])

        lo = gm.min_coord()
        hi = gm.max_coord()
        pairs = [((lo[0] + RESOLUTION, lo[1] + RESOLUTION), (hi[0] - RESOLUTION, hi[1] - RESOLUTION))]
        for _ in range(RANDOM_PAIRS_PER_MAP):
            s = random_free_point(gm, rng)
            g = random_free_point(gm, rng)
            if s and g:
                pairs.append((s, g))

        for start, goal in pairs:
            total_pairs += 1

            # bf: ground truth reachability + always-correct baseline path
            status, result, elapsed = run_bounded(bf, gm, *start, *goal, False)
            s = stats['bf']
            s['runs'] += 1
            s['max_time'] = max(s['max_time'], elapsed)
            s['total_time'] += elapsed
            bf_reachable = None
            if status == 'CRASH':
                s['crashes'] += 1
                bugs.append(f'map={map_id} bf CRASHED for start={start} goal={goal}:\n{result}')
                continue
            elif status == 'TIMEOUT':
                s['timeouts'] += 1
                bugs.append(f'map={map_id} bf TIMED OUT for start={start} goal={goal}')
                continue
            else:
                path, _search = result
                ok, err = validate_path(gm, path, start, goal)
                bf_reachable = bool(path)
                if bf_reachable:
                    s['found'] += 1
                else:
                    s['no_path'] += 1
                if not ok:
                    s['invalid_paths'] += 1
                    bugs.append(f'map={map_id} bf INVALID PATH start={start} goal={goal}: {err}')

            # grid planners: astar/dijkstra, 4- and 8-connectivity
            for name, fn, eight in (('astar_4', astar, False), ('astar_8', astar, True),
                                     ('dijkstra_4', dijkstra, False), ('dijkstra_8', dijkstra, True)):
                status, result, elapsed = run_bounded(fn, gm, *start, *goal, eight)
                s = stats[name]
                s['runs'] += 1
                s['max_time'] = max(s['max_time'], elapsed)
                s['total_time'] += elapsed
                if status == 'CRASH':
                    s['crashes'] += 1
                    bugs.append(f'map={map_id} {name} CRASHED start={start} goal={goal}:\n{result}')
                    continue
                if status == 'TIMEOUT':
                    s['timeouts'] += 1
                    bugs.append(f'map={map_id} {name} TIMED OUT start={start} goal={goal}')
                    continue
                path, _search = result
                ok, err = validate_path(gm, path, start, goal)
                if path:
                    s['found'] += 1
                else:
                    s['no_path'] += 1
                    if bf_reachable:
                        bugs.append(f'map={map_id} {name} found NO PATH but bf found one, '
                                    f'start={start} goal={goal} (search completeness bug, not the '
                                    f'known cost()/heuristic() TODO stubs)')
                if not ok:
                    s['invalid_paths'] += 1
                    bugs.append(f'map={map_id} {name} INVALID PATH start={start} goal={goal}: {err}')

            # sampling planners: rrt/rrtstar, repeated (stochastic)
            for name, fn in (('rrt', rrt), ('rrtstar', rrtstar)):
                for _rep in range(RRT_REPEATS):
                    status, result, elapsed = run_bounded(
                        fn, gm, *start, *goal,
                        iterations=RRT_ITERATIONS, max_edge_length=RRT_MAX_EDGE, early_stop=False)
                    s = stats[name]
                    s['runs'] += 1
                    s['max_time'] = max(s['max_time'], elapsed)
                    s['total_time'] += elapsed
                    if status == 'CRASH':
                        s['crashes'] += 1
                        bugs.append(f'map={map_id} {name} CRASHED start={start} goal={goal}:\n{result}')
                        continue
                    if status == 'TIMEOUT':
                        s['timeouts'] += 1
                        bugs.append(f'map={map_id} {name} TIMED OUT start={start} goal={goal}')
                        continue
                    path, _search = result
                    ok, err = validate_path(gm, path, start, goal)
                    if path:
                        s['found'] += 1
                    else:
                        s['no_path'] += 1
                    if not ok:
                        s['invalid_paths'] += 1
                        bugs.append(f'map={map_id} {name} INVALID PATH start={start} goal={goal}: {err}')

        print(f'map {map_id} done ({len(pairs)} start/goal pairs)', file=sys.stderr)

    print(f'\n=== SUMMARY ({total_pairs} start/goal pairs across {len(MAP_IDS)} maps) ===')
    for name, s in stats.items():
        avg = s['total_time'] / s['runs'] if s['runs'] else 0.0
        print(f'{name:12s} runs={s["runs"]:4d} found={s["found"]:4d} no_path={s["no_path"]:4d} '
              f'crashes={s["crashes"]} timeouts={s["timeouts"]} invalid_paths={s["invalid_paths"]} '
              f'avg_time={avg*1000:.1f}ms max_time={s["max_time"]*1000:.1f}ms')

    print(f'\n=== BUGS/ANOMALIES ({len(bugs)}) ===')
    for b in bugs:
        print(f'- {b}')

    print('\nRELIABILITY_TEST_RESULT=' + ('PASS' if not bugs else 'FAIL'))


if __name__ == '__main__':
    main()
