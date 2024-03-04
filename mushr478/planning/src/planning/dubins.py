"""Dubins path code (https://en.wikipedia.org/wiki/Dubins_path), originally
authored by Atsushi Sakai (@Atsushi_twi).

Modified by Aditya Vamsikrishna (adityavk), Gilwoo Lee (gilwoo), and Nick Walker
(nswalker) for CSE 478.
"""

import math

import matplotlib.pyplot as plt
import numpy as np
from cse478 import utils
import warnings


def path_planning(start, end, curvature, resolution=0.1, interpolate_line=True):
    """Return the Dubins path between two states.

    Args:
        start: list of (sx, sy, stheta)
            sx: x position of start state [m]
            sy: y position of start state [m]
            stheta: yaw angle of start state [rad]
        end: list of (ex, ey, etheta)
            ex: x position of end state [m]
            ey: y position of end state [m]
            etheta: yaw angle of end state [rad]
        curvature: Dubins path curvature [1/m]
        resolution: interpolation resolution

    Returns:
        path: interpolated Dubins path between start and end
        length: length of Dubins path
    """
    # Transform to make start of path the origin
    start = np.array(start, dtype=float)
    end = np.array(end, dtype=float)
    end[:2] -= start[:2]
    start_orientation = start[2]
    rotation = utils.rotation_matrix(start_orientation)
    end[:2] = np.matmul(end[:2], rotation)
    end[2] -= start[2]

    path, mode, length = path_planning_from_origin(
        end, curvature, resolution=resolution, interpolate_line=interpolate_line
    )
    # Transform back into original reference frame
    inv_rot = utils.rotation_matrix(-start_orientation)
    path[:, :2] = np.matmul(path[:, :2], inv_rot) + start[:2]
    path[:, 2] += start[2]
    pi_2_pi(path[:, 2])

    # Turn this back on if you see unnecessary rotations
    # path, length = process_dubins(start, path, length)

    # FIXME(nickswalker5-17-21): The dubins stuff is littered with this
    # "scale by curvature." It can probably be refactored out
    real_path_length = length * 1 / curvature
    return path, real_path_length


def path_length(start, end, curvature):
    """Return the length of the Dubins path between pairs of states.

    Args:
        start: start configurations shape (N, 3)
        end: end configurations shape (N, 3)
        curvature: shape N

    Returns:
        length: lengths of Dubins paths shape N
    """
    # Transform to make start of path the origin
    start = np.atleast_2d(start)
    end = np.atleast_2d(end)
    # We'll gracefully handle mismatched argument dimensions
    # in some situations where broadcasting semantics apply;
    # if either start or end have a singleton dimension, they'll
    # be broadcast to match the larger argument.
    end_broad = np.empty(np.broadcast(start, end).shape)
    end_broad[:, :2] = end[:, :2] - start[:, :2]
    end_broad[:, 2] = end[:, 2]
    start_orientation = start[:, 2]
    rotation = utils.rotation_matrix(start_orientation)
    rotation = np.moveaxis(rotation, -1, 0)
    end_broad[:, :2] = np.matmul(end_broad[:, :2][:, np.newaxis], rotation).squeeze()
    end_broad[:, 2] -= start[:, 2]

    _, _, cost = get_best_plan_from_origin(end_broad, curvature)
    return cost * (1 / curvature)


##########################
# Implementation Details #
##########################


def mod2pi(theta, out=None):
    if out is None:
        return theta - 2.0 * np.pi * np.floor(theta / 2.0 / np.pi)
    else:
        out[:] = theta - 2.0 * np.pi * np.floor(theta / 2.0 / np.pi)


def pi_2_pi(angles):
    """
    Wrap angles to (-pi, pi]
    Args:
        angles:
    """
    angles[:] = (angles[:] - np.pi) % (2 * np.pi) + np.pi


planner_modes = ["LSL", "RSR", "LSR", "RSL", "RLR", "LRL"]


def planner(alpha, beta, d):
    """
    Args:
        alpha: shape N
        beta:  shape N
        d: shape N

    Returns: np.array of path parameters with shape (N, 6, 3) where
                    the second dimension indexes the planner mode and
                    the third dimension indexes the segment parameter.
                    Turning segments are in radians and lines are in
                    distance units.
    """
    out = np.empty((len(alpha), 6, 3))
    sa = np.sin(alpha)
    sb = np.sin(beta)
    ca = np.cos(alpha)
    cb = np.cos(beta)
    c_ab = np.cos(alpha - beta)

    # LSL
    tmp0 = d + sa - sb
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb))
    tmp1 = np.arctan2((cb - ca), tmp0)
    out[:, 0, 0] = -alpha + tmp1
    out[:, 0, 1] = np.sqrt(p_squared)
    out[:, 0, 2] = beta - tmp1

    # RSR
    tmp0 = d - sa + sb
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa))
    tmp1 = np.arctan2((ca - cb), tmp0)
    out[:, 1, 0] = alpha - tmp1
    out[:, 1, 1] = np.sqrt(p_squared)
    out[:, 1, 2] = -beta + tmp1

    # LSR
    p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb))
    out[:, 2, 1] = np.sqrt(p_squared)
    p = out[:, 2, 1]
    tmp2 = np.arctan2((-ca - cb), (d + sa + sb)) - np.arctan2(-2.0, p)
    out[:, 2, 0] = -alpha + tmp2
    out[:, 2, 2] = -mod2pi(beta) + tmp2

    # RSL
    p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb))
    out[:, 3, 1] = np.sqrt(p_squared)
    p = out[:, 3, 1]
    tmp2 = np.arctan2((ca + cb), (d - sa - sb)) - np.arctan2(2.0, p)
    out[:, 3, 0] = alpha - tmp2
    out[:, 3, 2] = beta - tmp2

    # RLR
    tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
    out[:, 4, 1] = mod2pi(2 * np.pi - np.arccos(tmp_rlr))
    p = out[:, 4, 1]
    out[:, 4, 0] = alpha - np.arctan2(ca - cb, d - sa + sb) + mod2pi(p / 2.0)
    rlr_t = out[:, 4, 0]
    out[:, 4, 2] = alpha - beta - rlr_t + mod2pi(p)

    # LRL
    tmp_lrl = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (-sa + sb)) / 8.0
    out[:, 5, 1] = mod2pi(2 * np.pi - np.arccos(tmp_lrl))
    p = out[:, 5, 1]
    out[:, 5, 0] = -alpha - np.arctan2(ca - cb, d + sa - sb) + p / 2.0
    lrl_t = out[:, 5, 0]
    out[:, 5, 2] = mod2pi(beta) - alpha - lrl_t + mod2pi(p)

    mod2pi(out[:, :, 0], out=out[:, :, 0])
    mod2pi(out[:, :, 2], out=out[:, :, 2])
    return out


def get_best_plan_from_origin(goal, curvature):
    # nomalize
    d = np.sqrt(goal[:, 0] ** 2.0 + goal[:, 1] ** 2.0)
    d = d * curvature
    #  print(dx, dy, D, d)

    theta = mod2pi(np.arctan2(goal[:, 1], goal[:, 0]))
    alpha = mod2pi(-theta)
    beta = mod2pi(goal[:, 2] - theta)
    #  print(theta, alpha, beta, d)

    # Some configurations can't be connected with some (or all)
    # Dubins path configurations. This will manifest as domain errors
    # in calls to trig functions. Numpy gracefully propogates NaNs
    # and we will hide the warning because the code can handle them
    # just fine.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        all_plans = planner(alpha, beta, d)
    all_i = np.arange(len(all_plans))
    cost = np.abs(all_plans).sum(-1)
    best_i = np.nanargmin(cost, axis=1)
    best_plan = all_plans[all_i, best_i]
    best_cost = cost[all_i, best_i]
    return best_plan, list(map(lambda x: planner_modes[x], best_i)), best_cost


def path_planning_from_origin(goal, curvature, resolution=0.1, interpolate_line=False):
    if np.all(np.array(goal) == 0.0):
        return np.zeros((1, 3)), [], 0.0

    lengths, modes, cost = get_best_plan_from_origin(np.atleast_2d(goal), curvature)

    path = generate_course(
        lengths.squeeze(),
        modes,
        curvature,
        step_size=resolution,
        interpolate_line=interpolate_line,
    )
    return path, modes, cost


def line(length=10.0, waypoint_sep=0.1):
    # Desired waypoint_sep may not divide the length evenly.
    # In that case, we'll round up (add at most one extra waypoint),
    # but still hit the length exactly.
    # Need at least a start and end (2 points)
    num_waypoints = max(math.ceil(length / waypoint_sep), 2)
    line_coords = np.mgrid[0 : length : complex(num_waypoints)]
    return np.vstack([line_coords, np.zeros((2, len(line_coords)))]).T


def turn(radius, angle, waypoint_sep=0.1):
    turn_center = [0, radius]
    arclength = angle * radius
    # Need at least a start and end (2 points)
    num_turn_points = max(math.ceil(arclength / waypoint_sep), 2)
    thetas = np.mgrid[0 : angle : complex(num_turn_points)]
    thetas -= np.pi / 2
    turn_poses = np.empty((len(thetas), 3))
    turn_poses[:, 0] = radius * np.cos(thetas) + turn_center[0]
    turn_poses[:, 1] = radius * np.sin(thetas) + turn_center[1]
    turn_poses[:, 2] = thetas + (np.pi / 2)
    return turn_poses


def generate_course(lengths, mode, curvature, step_size=0.1, interpolate_line=False):
    segments = []
    for m, length in zip(mode[0], lengths):
        # 0 length segments are just a noop
        if length == 0.0:
            continue
        if m == "S":
            if interpolate_line:
                segments.append(line(length / curvature, step_size))
            else:
                segments.append(np.array([[0, 0, 0], [length / curvature, 0, 0]]))
        elif m == "L":
            segments.append(turn(1 / curvature, length, step_size))
        elif m == "R":
            segments.append(turn(1 / curvature, length, step_size) * [1, -1, -1])
        else:
            raise RuntimeError("Unexpected mode type '{}'".format(m))

    # Each of the segments starts at the origin. String
    # them together by transforming each to start
    # at the previous' endpoint
    for i in range(1, len(segments)):
        seg = segments[i]
        start = segments[i - 1][-1]
        rot = utils.rotation_matrix(-start[2])
        seg[:, :2] = np.matmul(seg[:, :2], rot) + start[:2]
        seg[:, 2] += start[2]
    return np.vstack(segments)


def process_dubins(start, path, cost):
    """Ensure no 2pi rotations in the output due to numerical issues."""
    eps = 1e-6
    close_to_start = np.sum(abs(path - start) < eps, axis=1) == 3
    ind = np.where(close_to_start[1 : len(path) - 1])[0]
    if len(ind) > 0:
        return path[ind[0] + 1 :, :], cost - math.radians(360.0)

    return path, cost


