#!/usr/bin/env python

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from assignment_3.grid_map import GridMap


def reconstruct_path(came_from, goal):
    path = [goal]
    cur = goal
    while cur in came_from:
        cur = came_from[cur]
        path.append(cur)
    path.reverse()
    return path


def cell_to_coord_path(gm: GridMap, path):
    e_path = []
    for e in path:
        e_path.append(gm.coord(*e))
    return e_path


def to_ros_path(path: list[tuple[float, float]], frame: str = '', stamp=None) -> Path:
    msg = Path()
    msg.header.frame_id = frame
    if stamp:
        msg.header.stamp = stamp
    for p in path:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = 0.1
        msg.poses.append(pose)
    return msg
