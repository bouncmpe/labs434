"""Your controller for the course project: edit this file, not simulate.py.

simulate.py creates one Controller after building the dungeon, then calls control() once per
physics step (every 2 ms) to get the wheel speeds. Everything the robot may use is here:

    m, d   MuJoCo model and data. Names of robot parts start with "robot-", e.g.
           d.sensor("robot-laser00").data   laser ranges, 19 beams from right (00) to left (18),
                                            10 degrees apart, -1 when nothing is within 10 m
           d.sensor("robot-sonar_left").data, "robot-sonar_back", "robot-sonar_right" (4 m)
           d.sensor("robot-imu_accel").data, "robot-imu_gyro", "robot-imu_quat"
           d.body("robot-base_link").xpos, .xquat   pose of the base
           d.site("finish").xpos                    center of the finish tile
           d.time                                   simulation time (s)
    tiles  the dungeon map, a dict from (i, j) to "." floor, "#" wall or " " outside. Tile (i, j)
           is the 2 m square centered at x = 2 i, y = 2 j (see TILE).

Wheel speeds are in rad/s within +-15.3, i.e. 1.5 m/s at the rim; larger values are clamped.
"""

import numpy as np

TILE = 2.0  # m, side length of a dungeon tile
WHEEL_RADIUS = 0.098  # m
TRACK = 0.4044  # m, distance between the two drive wheels


def wheel_speeds(v, w):
    """Return the left and right wheel speeds (rad/s) for forward speed v (m/s) and turn rate w (rad/s)."""
    return (v - w * TRACK / 2) / WHEEL_RADIUS, (v + w * TRACK / 2) / WHEEL_RADIUS


class Controller:
    def __init__(self, m, d, tiles):
        """Called once before the simulation starts: plan here, e.g. a path from start to finish."""
        self.tiles = tiles
        self.goal = d.site("finish").xpos[:2].copy()
        self.laser = [d.sensor(f"robot-laser{i:02d}") for i in range(19)]

    def control(self, m, d):
        """Called every physics step: return the (left, right) wheel speeds in rad/s.

        Placeholder: wander forward and turn left when something is close ahead. It does not
        look for the finish; replace it with your own planner and controller.
        """
        ranges = np.array([s.data[0] for s in self.laser])
        ranges[ranges < 0] = np.inf  # nothing within range
        ahead = ranges[6:13].min()  # the 7 beams within 30 degrees of straight ahead

        if ahead < 0.6:
            return wheel_speeds(0.0, 1.0)  # turn in place
        return wheel_speeds(0.4, 0.0)
