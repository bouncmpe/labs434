"""Course project: drive the TIAGo base from the start tile to the finish tile of a random dungeon.

Usage (from the repository root):
    python project/simulate.py [--seed SEED] [--controller MODULE]

To score a controller over many dungeons without the viewer, see evaluate.py.

The dungeon is a grid of rooms joined by corridors (see dungeon.py), each tile 2 m wide. The
start tile is marked blue and the finish tile red. Every room holds one moving cylindrical
obstacle that bounces off the walls. Press SPACE in the viewer to pause and resume.

The robot is driven by the Controller class in controller.py (or in the module given by
--controller, e.g. my_solution for project/my_solution.py). Write your controller there: set the
wheel speeds from the robot's sensors and the map so it reaches the finish without hitting walls
or obstacles.
"""

import argparse
import importlib
import random
import time

import mujoco
import mujoco.viewer
import numpy as np

import dungeon

SCENE = "scenes/empty_floor.xml"
ROBOT = "robots/pal_tiago_base/tiago.xml"
TILE = 2.0  # m, side length of a dungeon tile
CELLS_X, CELLS_Y = 4, 3  # dungeon size in cells, one room per cell
CELL_SIZE = 8  # tiles per cell side: the dungeon is CELLS_X * CELL_SIZE * TILE m wide (64 m)
WALL_HEIGHT = 0.5  # m, tall enough for the laser (0.09 m) and sonars (0.27 m) to see
OBSTACLE_RADIUS = 0.35  # m
OBSTACLE_HEIGHT = 0.3  # m
OBSTACLE_SPEED = 0.5  # m/s
GOAL_RADIUS = 0.5  # m, the finish is reached when the base center is this close to the finish center
TIME_LIMIT = 300  # s of wall-clock time before the viewer closes

paused = False


def key_callback(keycode):
    """Toggle the paused state when SPACE is pressed."""
    global paused
    if keycode == ord(" "):
        paused = not paused


def tile_center(tile):
    return [tile[0] * TILE, tile[1] * TILE]


def build_scene():
    """Return the dungeon scene spec with the robot attached, and the tile map."""
    spec = mujoco.MjSpec.from_file(SCENE)
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST  # as in tiago.xml
    # Far clipping distance is zfar * extent (0.6 m in the scene): 200 gives 120 m, the whole dungeon.
    spec.visual.map.zfar = 200

    tiles, rooms, connections = dungeon.generate(CELLS_X, CELLS_Y, CELL_SIZE)

    # Room floors are drawn slightly above the ground plane and do not collide.
    for index, room in enumerate(rooms):
        xmin, ymin, xmax, ymax = dungeon.find_room_corners(room)
        spec.worldbody.add_geom(
            name=f"R{index}",
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            size=[(xmax - xmin + 1) * TILE / 2, (ymax - ymin + 1) * TILE / 2, 0.1],
            pos=[(xmin + xmax) * TILE / 2, (ymin + ymax) * TILE / 2, 0.001],
            rgba=[0.8, 0.6, 0.4, 1],
            contype=0,
            conaffinity=0,
        )

    for tile, kind in tiles.items():
        if kind == "#":
            spec.worldbody.add_geom(
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[TILE / 2, TILE / 2, WALL_HEIGHT / 2],
                pos=[*tile_center(tile), WALL_HEIGHT / 2],
                rgba=[0.8, 0.6, 0.4, 1],
            )

    floor = [tile for tile, kind in tiles.items() if kind == "."]
    start, finish = random.sample(floor, 2)

    for name, tile, rgba in [("start", start, [0, 0, 1, 1]), ("finish", finish, [1, 0, 0, 1])]:
        spec.worldbody.add_site(
            name=name,
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[TILE / 4, TILE / 4, 0.01],
            pos=[*tile_center(tile), 0],
            rgba=rgba,
        )

    # One moving obstacle per room (corridors included), on a mocap body so the code below can move it.
    for index, room in enumerate(rooms):
        candidates = [tile for tile in room if tile not in (start, finish)]
        if not candidates:
            continue
        body = spec.worldbody.add_body(
            name=f"Z{index}", mocap=True, pos=[*tile_center(random.choice(candidates)), OBSTACLE_HEIGHT / 2]
        )
        body.add_geom(
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=[OBSTACLE_RADIUS, OBSTACLE_HEIGHT / 2, 0],
            rgba=[0.8, 0.0, 0.1, 1],
        )

    # Attach the robot at the start tile with a random heading. A prefix keeps its names apart
    # from the scene's (and would let us add more robots from the same model).
    robot = mujoco.MjSpec.from_file(ROBOT)
    spec.attach(robot, frame="world", prefix="robot-")
    base = spec.body("robot-base_link")
    base.pos[:2] = tile_center(start)
    yaw = np.random.uniform(-np.pi, np.pi)
    base.quat[:] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]

    return spec, tiles


def move_obstacles(m, d, obstacles, directions, dt):
    """Move each obstacle along its direction, bouncing off walls like a billiard ball.

    The walls are axis-aligned boxes, so a bounce flips the x or y component of the direction.
    For each axis, three parallel rays (from the center and both edges of the cylinder) find
    whether that side is blocked. The edge rays catch wall corners, e.g. at doorways, that a
    single center ray would miss. Rays also hit the robot and other obstacles.
    """
    geomid = np.zeros(1, dtype=np.int32)
    reach = OBSTACLE_RADIUS + 0.05  # m, bounce this close to a surface

    for k, body in enumerate(obstacles):
        mocap = m.body_mocapid[body]
        center = d.mocap_pos[mocap].copy()
        center[2] = 0.1  # below the wall tops, above the floor
        direction = directions[k]

        for axis in (0, 1):
            if direction[axis] == 0:
                continue
            ray = np.zeros(3)
            ray[axis] = np.sign(direction[axis])
            side = np.zeros(3)
            side[1 - axis] = OBSTACLE_RADIUS
            for origin in (center, center + side, center - side):
                distance = mujoco.mj_ray(m, d, origin, ray, None, 1, body, geomid)
                if 0 <= distance < reach:
                    direction[axis] = -direction[axis]
                    break

        d.mocap_pos[mocap] += direction * OBSTACLE_SPEED * dt


class Simulation:
    """One dungeon with the robot, its controller and the moving obstacles, without a viewer."""

    def __init__(self, controller="controller", seed=None):
        """Build a dungeon (the same one for the same seed) and create the Controller from the given module."""
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)

        spec, self.tiles = build_scene()

        # m holds the static model information and d the dynamic state.
        self.m = m = spec.compile()
        self.d = d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)  # fill in positions and sensors before the controller sees them

        self.obstacles = [m.body(i).id for i in range(m.nbody) if m.body(i).name.startswith("Z")]
        angles = np.random.uniform(-np.pi, np.pi, len(self.obstacles))
        self.directions = np.stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)], axis=1)

        # The wheel actuators are named in tiago.xml, here with the prefix added at attach time.
        self.left = d.actuator("robot-wheel_left_joint_vel")
        self.right = d.actuator("robot-wheel_right_joint_vel")
        self.base = d.body("robot-base_link")
        self.finish = d.site("finish").xpos[:2].copy()

        self.controller = importlib.import_module(controller).Controller(m, d, self.tiles)

    def step(self):
        """Ask the controller for the wheel speeds, move the obstacles and advance the physics one timestep."""
        self.left.ctrl, self.right.ctrl = self.controller.control(self.m, self.d)
        move_obstacles(self.m, self.d, self.obstacles, self.directions, self.m.opt.timestep)
        mujoco.mj_step(self.m, self.d)

    def reached_finish(self):
        return np.linalg.norm(self.base.xpos[:2] - self.finish) < GOAL_RADIUS


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seed", type=int, help="random seed for a repeatable dungeon (default: random)")
    parser.add_argument("--controller", default="controller", help="module with the Controller class (default: controller)")
    args = parser.parse_args()

    sim = Simulation(args.controller, args.seed)
    m, d = sim.m, sim.d
    reported = False

    with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = m.body("robot-base_link").id
        viewer.cam.distance = 4.0
        viewer.cam.elevation = -40

        start_time = time.time()
        while viewer.is_running() and time.time() - start_time < TIME_LIMIT:
            step_start = time.time()

            if not paused:
                sim.step()
                if not reported and sim.reached_finish():
                    print(f"Reached the finish at t = {d.time:.1f} s")
                    reported = True

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()

            # Rudimentary time keeping, will drift relative to wall clock.
            remaining = m.opt.timestep - (time.time() - step_start)
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()
