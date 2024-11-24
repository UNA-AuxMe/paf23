from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from math import sqrt

from typing import Tuple

import numpy as np
from numpy.typing import NDArray


def pose_to_position_distance(point: Point, position: Point) -> float:
    """
    2D Distance between the given point and the given position
    :param point: The point to measure from
    :param position: The position to measure to
    :return: distance
    """
    x_cur = position.x
    y_cur = position.y
    x_target = point.x
    y_target = point.y
    d = (x_target - x_cur) ** 2 + (y_target - y_cur) ** 2
    return sqrt(d)


def get_closest_point_index(path: Path, position: Point) -> int:
    """
    Returns index of the nearest point of the path in 2D (height omitted)
    :return: Index of the closest point
    """
    if len(path.poses) < 2:
        return -1

    min_dist = 10e100
    min_dist_idx = -1

    for i in range(0, len(path.poses)):
        temp_pose: PoseStamped = path.poses[i]
        dist = pose_to_position_distance(temp_pose.pose.position, position)
        if min_dist > dist:
            min_dist = dist
            min_dist_idx = i
    return min_dist_idx


def point_to_2D_ndarray(point: Point) -> NDArray[np.float_]:
    return np.array([point.x, point.y])


def get_2D_direction(point_a: Point, point_b: Point) -> NDArray[np.float_]:
    """Returns the direction vector from point a to point b

    Args:
        point_a (Point): _description_
        point_b (Point): _description_

    Returns:
        NDArray[np.float_]: _description_
    """
    return np.array(
        [
            point_b.x - point_a.x,
            point_b.y - point_a.y,
        ]
    )


def is_same_direction(
    point: Point, position: Point, direction: NDArray[np.float_]
) -> bool:
    """Returns true if the angle between direction and postion_to_point is smaller
    than 90 deg

    aka. same direction

    Args:
        point (Point): _description_
        position (Point): _description_
        orientation (NDArray[np.float_]): _description_

    Returns:
        bool: _description_
    """
    to_point: NDArray = get_2D_direction(position, point)
    direction: NDArray = direction[:2]
    return np.dot(to_point, direction) > 0


def get_closest_forward_index(
    path: Path, position: Point, direction: NDArray[np.float_]
) -> int:
    """Returns the index of the closest point in driving direction.

    Args:
        path (Path): _description_
        position (Point): _description_

    Returns:
        int: _description_
    """

    if len(path.poses) < 2:
        return -1

    min_dist = 10e100
    min_dist_idx = -1

    for i in range(0, len(path.poses)):
        temp_pose: PoseStamped = path.poses[i]
        dist = pose_to_position_distance(temp_pose.pose.position, position)
        if (
            is_same_direction(temp_pose.pose.position, position, direction)
            and min_dist > dist
        ):
            min_dist = dist
            min_dist_idx = i
    return min_dist_idx
