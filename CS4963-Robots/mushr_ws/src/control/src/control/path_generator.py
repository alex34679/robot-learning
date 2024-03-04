"""Simple parametric paths."""
from __future__ import division
import numpy as np


def line(length=10.0, waypoint_sep=0.1):
    line_coords = np.mgrid[0:length:waypoint_sep]
    return np.vstack(
        [line_coords, np.zeros_like(line_coords), np.zeros_like(line_coords)]
    ).T


def saw(scale=3, n=2, waypoint_sep=0.1):
    # Go from 0 to 5 in steps of .05
    duration = 2
    # We move at a 45 degree angle, so scale by sqrt(2) to keep waypoint separation
    triangle_step = waypoint_sep * (2 ** -0.5)
    # We need to modify our step factor to account for the inflation
    # we do at the end with the scale factor
    t = np.mgrid[0 : duration : triangle_step / scale]
    saw = _sawtooth(np.pi * (t - 1.0), 1)
    # Find the tip of the tooth
    diff = np.diff(saw)
    discon_idx = np.where(abs(diff) > (waypoint_sep + 0.01))[0] + 1
    discon_idx = discon_idx[0]
    saw_path = np.vstack([t, saw, np.full_like(t, np.pi / 4)]).T
    # Draw in a downward line to fix the discontinuity
    vertical = np.mgrid[
        saw_path[discon_idx - 1, 1] : saw_path[discon_idx, 1] : -waypoint_sep / scale
    ][:-1]
    vertical_path = np.vstack(
        [
            np.full_like(vertical, t[discon_idx]),
            vertical,
            np.full_like(vertical, -np.pi / 2),
        ]
    ).T
    patched = np.vstack([saw_path[:discon_idx], vertical_path, saw_path[discon_idx:]])

    path = np.tile(patched.T, n).T
    path[:, 0] += duration * (np.arange(len(path)) // len(patched))
    # Increase the height and width of the saw
    path[:, [0, 1]] *= scale
    return path


def wave(amplitude=1.0, n=2, waypoint_sep=0.1):
    # It's not easy to calculate the arclength of a sine wave:
    # https://math.stackexchange.com/questions/45089/what-is-the-length-of-a-sine-wave-from-0-to-2-pi
    # so spacing is merely proportional to waypoint_sep
    t = np.mgrid[0 : 2 * np.pi * n : waypoint_sep]
    configs = np.empty((len(t), 3))
    configs[:, 0] = t
    configs[:, 1] = np.sin(t) * amplitude
    configs[:, 2] = np.cos(t)

    return configs


def circle(radius=2.5, waypoint_sep=0.1):
    center = [0, radius]
    num_points = (2 * radius * np.pi) / waypoint_sep
    thetas = np.mgrid[-np.pi / 2 : 2 * np.pi - np.pi / 2 : 2 * np.pi / num_points]
    poses = np.empty((len(thetas), 3))
    poses[:, 0] = radius * np.cos(thetas) + center[0]
    poses[:, 1] = radius * np.sin(thetas) + center[1]
    poses[:, 2] = np.pi / 2 + thetas

    # Drop a few points off the end so that the controller doesn't overshoot and
    # end up restarting the loop
    return poses[: -max(int(0.5 / waypoint_sep), 1)]


def left_turn(waypoint_sep=0.1, turn_radius=1.5, straight_len=5.0):
    turn_center = [
        straight_len + turn_radius * np.cos(np.pi / 2),
        turn_radius,
    ]
    straight_xs = np.mgrid[0:straight_len:waypoint_sep]
    straight_poses = np.vstack(
        [straight_xs, np.zeros_like(straight_xs), np.zeros_like(straight_xs)]
    ).T
    num_turn_points = (2 * turn_radius * np.pi * 0.5) / waypoint_sep
    thetas = np.mgrid[-np.pi / 2 : 0 : 2 * np.pi / num_turn_points]

    turn_poses = np.empty((len(thetas), 3))
    turn_poses[:, 0] = turn_radius * np.cos(thetas) + turn_center[0]
    turn_poses[:, 1] = turn_radius * np.sin(thetas) + turn_center[1]
    turn_poses[:, 2] = thetas + (np.pi / 2)

    straight_after = np.empty((10, 3))
    straight_after[:, 0] = turn_poses[-1, 0]
    straight_after[:, 1] = turn_poses[-1, 1] + np.mgrid[0:1:0.1]
    straight_after[:, 2] = np.pi / 2
    return np.vstack([straight_poses, turn_poses[:-1], straight_after])


def right_turn(waypoint_sep=0.1, turn_radius=1.5, straight_len=5.0):
    turn_center = [
        straight_len + turn_radius * np.cos(np.pi / 2),
        -turn_radius,
    ]
    straight_xs = np.mgrid[0:straight_len:waypoint_sep]
    straight_poses = np.vstack(
        [straight_xs, np.zeros_like(straight_xs), np.zeros_like(straight_xs)]
    ).T
    num_turn_points = (2 * turn_radius * np.pi * 0.5) / waypoint_sep
    thetas = np.mgrid[np.pi / 2 : 0 : -2 * np.pi / num_turn_points]

    turn_poses = np.empty((len(thetas), 3))
    turn_poses[:, 0] = turn_radius * np.cos(thetas) + turn_center[0]
    turn_poses[:, 1] = turn_radius * np.sin(thetas) + turn_center[1]
    turn_poses[:, 2] = thetas - (np.pi / 2)

    straight_after = np.empty((10, 3))
    straight_after[:, 0] = turn_poses[-1, 0]
    straight_after[:, 1] = turn_poses[-1, 1] - np.mgrid[0:1:0.1]
    straight_after[:, 2] = -np.pi / 2
    return np.vstack([straight_poses, turn_poses[:-1], straight_after])


def _sawtooth(t, width=1):
    """
    Return a periodic sawtooth or triangle waveform.
    The sawtooth waveform has a period ``2*pi``, rises from -1 to 1 on the
    interval 0 to ``width*2*pi``, then drops from 1 to -1 on the interval
    ``width*2*pi`` to ``2*pi``. `width` must be in the interval [0, 1].
    Note that this is not band-limited.  It produces an infinite number
    of harmonics, which are aliased back and forth across the frequency
    spectrum.
    Parameters
    ----------
    t : array_like
        Time.
    width : array_like, optional
        Width of the rising ramp as a proportion of the total cycle.
        Default is 1, producing a rising ramp, while 0 produces a falling
        ramp.  `width` = 0.5 produces a triangle wave.
        If an array, causes wave shape to change over time, and must be the
        same length as t.
    Returns
    -------
    y : ndarray
        Output array containing the sawtooth waveform.
    Examples
    --------
    A 5 Hz waveform sampled at 500 Hz for 1 second:
    """
    t, w = np.asarray(t), np.asarray(width)
    w = np.asarray(w + (t - t))
    t = np.asarray(t + (w - w))
    if t.dtype.char in ["fFdD"]:
        ytype = t.dtype.char
    else:
        ytype = "d"
    y = np.zeros(t.shape, ytype)

    # width must be between 0 and 1 inclusive
    mask1 = (w > 1) | (w < 0)
    np.place(y, mask1, np.nan)

    # take t modulo 2*pi
    tmod = np.mod(t, 2 * np.pi)

    # on the interval 0 to width*2*pi function is
    #  tmod / (pi*w) - 1
    mask2 = (1 - mask1) & (tmod < w * 2 * np.pi)
    tsub = np.extract(mask2, tmod)
    wsub = np.extract(mask2, w)
    np.place(y, mask2, tsub / (np.pi * wsub) - 1)

    # on the interval width*2*pi to 2*pi function is
    #  (pi*(w+1)-tmod) / (pi*(1-w))

    mask3 = (1 - mask1) & (1 - mask2)
    tsub = np.extract(mask3, tmod)
    wsub = np.extract(mask3, w)
    np.place(y, mask3, (np.pi * (wsub + 1) - tsub) / (np.pi * (1 - wsub)))
    return y
