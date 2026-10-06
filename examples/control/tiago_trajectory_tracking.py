"""Drive the TIAGo base along a figure-eight trajectory with a tracking controller.

Usage:
    python examples/control/tiago_trajectory_tracking.py [--speed SPEED] [--rate RATE] [--start {1,2,3}]

Examples:
    python examples/control/tiago_trajectory_tracking.py --speed 0.2
    python examples/control/tiago_trajectory_tracking.py --speed 1.0 --rate 100
    python examples/control/tiago_trajectory_tracking.py --start 3

SPEED is the peak speed of the base as a fraction of its 1.5 m/s top speed, reached where
the figure-eight crosses itself. Tracking stays within a few centimeters up to about 0.75;
faster, the outer wheel hits its limit in the tight turns at both ends and the base falls
behind the reference.

RATE is the simulation step rate (Hz). Each simulation step reads the pose, runs the controller
once, holds its wheel commands while the physics advances 1 / RATE seconds in the model's
own timestep (2 ms in tiago.xml), and then updates the viewer.

START selects where the base is placed at t = 0: 1 on the reference (no error), 2 near it
(0.7 m off and turned 45 degrees) or 3 far from it (1.5 m away, facing the opposite way).

The reference path is drawn in white, the current reference point as an orange sphere and
the path traveled by the base in green. The plot shows the three tracking errors.
"""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--speed", type=float, default=0.4, help="peak speed as a fraction of the top speed (default: 0.4)")
parser.add_argument("--rate", type=float, default=50, help="simulation steps per second in Hz (default: 50)")
parser.add_argument("--start", type=int, choices=[1, 2, 3], default=2, help="start pose of the base (default: 2)")
args = parser.parse_args()
if not 0 < args.speed <= 1:
    parser.error("--speed must be in (0, 1]")
if args.rate <= 0:
    parser.error("--rate must be positive")

MODEL = "robots/pal_tiago_base/scene.xml"
MAX_SPEED = 1.5  # m/s, datasheet top speed of the base

# Reference: x = A sin(W t), y = A sin(W t) cos(W t). Its speed A W sqrt(cos^2(W t) + cos^2(2 W t))
# peaks at A W sqrt(2) where the path crosses itself, so W follows from the requested peak speed.
A = 2.0  # m, half the length of the figure-eight
W = args.speed * MAX_SPEED / (A * np.sqrt(2))
PERIOD = 2 * np.pi / W  # s, one loop

K_X, K_Y, K_THETA = 2.0, 8.0, 4.0  # tracking gains (1/s, 1/m^2, 1/m)
START_POSES = {  # x, y (m) and heading (rad) of the base at t = 0; the reference starts at 0, 0, pi/4
    1: (0.0, 0.0, np.pi / 4),  # on the reference
    2: (-0.5, -0.5, 0.0),  # near it
    3: (0.0, -1.5, np.pi),  # far from it, facing the opposite way
}
START_POSE = START_POSES[args.start]

PATH_SEGMENTS = 200  # line segments drawn for the reference path
TRACE_LENGTH = 600  # number of points kept of the traveled path
TRACE_EVERY = 2 * PERIOD / TRACE_LENGTH  # s between recorded points, so the trace covers two loops
HISTORY = 500  # number of samples shown in the plot, one per simulation step
SPEED = 1.0  # simulated seconds per real second, e.g. 2.0 to watch it twice as fast


def reference(t):
    """Return the reference pose (x, y, theta) and its speeds (v, w) at time t."""
    s, c = np.sin(W * t), np.cos(W * t)
    x, y = A * s, A * s * c
    dx, dy = A * W * c, A * W * np.cos(2 * W * t)  # first derivatives
    ddx, ddy = -A * W**2 * s, -2 * A * W**2 * np.sin(2 * W * t)  # second derivatives
    v = np.hypot(dx, dy)
    w = (dx * ddy - dy * ddx) / v**2  # curvature times speed
    return np.array([x, y, np.arctan2(dy, dx)]), v, w


def base_pose(d):
    """Return the planar pose (x, y, theta) of the base from its free joint."""
    x, y = d.body("base_link").xpos[:2]
    qw, qx, qy, qz = d.body("base_link").xquat
    return np.array([x, y, np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))])


def tracking_errors(pose, ref):
    """Return the reference pose relative to the base: ahead, to the left and heading."""
    dx, dy = ref[:2] - pose[:2]
    c, s = np.cos(pose[2]), np.sin(pose[2])
    e_theta = np.arctan2(np.sin(ref[2] - pose[2]), np.cos(ref[2] - pose[2]))  # wrapped to (-pi, pi]
    return np.array([c * dx + s * dy, -s * dx + c * dy, e_theta])


def controller(errors, v_ref, w_ref):
    """Kanayama tracking law: forward speed v (m/s) and turn rate w (rad/s)."""
    e_x, e_y, e_theta = errors
    v = v_ref * np.cos(e_theta) + K_X * e_x
    w = w_ref + v_ref * (K_Y * e_y + K_THETA * np.sin(e_theta))
    return v, w


m = mujoco.MjModel.from_xml_path(MODEL)
d = mujoco.MjData(m)

# Differential drive geometry: wheel radius from the actuator comment in tiago.xml, track
# width from the wheel body positions.
WHEEL_RADIUS = 0.098
TRACK = m.body("wheel_left_link").pos[1] - m.body("wheel_right_link").pos[1]
left = m.actuator("wheel_left_joint_vel").id
right = m.actuator("wheel_right_joint_vel").id

# Physics steps per simulation step: the model's timestep stays fine enough for the contacts.
SUBSTEPS = max(1, round(1 / (args.rate * m.opt.timestep)))

# Place the base at the start pose (heading as a rotation about the z axis).
x0, y0, theta0 = START_POSE
d.qpos[:2] = x0, y0
d.qpos[3:7] = np.cos(theta0 / 2), 0, 0, np.sin(theta0 / 2)
mujoco.mj_forward(m, d)

path = np.array([reference(t)[0][:2] for t in np.linspace(0, PERIOD, PATH_SEGMENTS + 1)])
trace = [base_pose(d)[:2]]
next_trace = TRACE_EVERY


def add_geom(scn, type, size, rgba):
    if scn.ngeom >= scn.maxgeom:
        return None
    geom = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(geom, type, size, np.zeros(3), np.eye(3).flatten(), rgba)
    scn.ngeom += 1
    return geom


def draw_polyline(scn, points, rgba, width):
    for p, q in zip(points[:-1], points[1:]):
        geom = add_geom(scn, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3), rgba)
        if geom is None:
            return
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, width, np.append(p, 0.01), np.append(q, 0.01))


def draw(scn, ref):
    scn.ngeom = 0
    draw_polyline(scn, path, np.array([1, 1, 1, 0.6]), 2)
    draw_polyline(scn, trace, np.array([0.2, 0.9, 0.2, 1]), 3)
    geom = add_geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.05, 0, 0]), np.array([1, 0.5, 0, 1]))
    if geom is not None:
        geom.pos = [ref[0], ref[1], 0.05]


# Figure setup: one line per tracking error.
fig = mujoco.MjvFigure()
mujoco.mjv_defaultFigure(fig)
fig.title = "tracking errors"
fig.xlabel = "time (s)"
fig.xformat = "%.1f"
fig.yformat = "%.2f"
for k, name in enumerate(["e_x (m)", "e_y (m)", "e_theta (rad)"]):
    fig.linename[k] = name
fig.flg_legend = 1
fig.flg_extend = 1
fig.figurergba[3] = 0.5  # semi-transparent background

samples = []  # (time, [e_x, e_y, e_theta])

with mujoco.viewer.launch_passive(m, d) as viewer:
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 0  # hide the laser and sonar rays

    start = time.time()
    while viewer.is_running():
        ref, v_ref, w_ref = reference(d.time)
        pose = base_pose(d)
        errors = tracking_errors(pose, ref)
        v, w = controller(errors, v_ref, w_ref)

        # Differential drive: each wheel rolls at the speed of its side of the base. Commands
        # beyond the servo ctrlrange (e.g. the large initial error) are clamped by MuJoCo.
        d.ctrl[left] = (v - w * TRACK / 2) / WHEEL_RADIUS
        d.ctrl[right] = (v + w * TRACK / 2) / WHEEL_RADIUS

        mujoco.mj_step(m, d, SUBSTEPS)

        if d.time >= next_trace:
            trace = (trace + [pose[:2]])[-TRACE_LENGTH:]
            next_trace += TRACE_EVERY

        samples.append((d.time, errors))
        samples = samples[-HISTORY:]

        # linedata holds x, y pairs: [x0, y0, x1, y1, ...].
        for k in range(3):
            for j, (t, values) in enumerate(samples):
                fig.linedata[k][2 * j] = t
                fig.linedata[k][2 * j + 1] = values[k]
            fig.linepnt[k] = len(samples)

        # Place the figure in the bottom-left corner of the 3D scene. viewer.viewport is the
        # scene area only; its left edge starts after the sidebar, so offset by left/bottom.
        scene = viewer.viewport
        if scene is not None:
            viewport = mujoco.MjrRect(scene.left, scene.bottom, scene.width // 3, scene.height // 3)
            viewer.set_figures((viewport, fig))

        with viewer.lock():
            draw(viewer.user_scn, ref)
        viewer.sync()

        # Wait until the wall clock catches up with the simulation.
        remaining = d.time / SPEED - (time.time() - start)
        if remaining > 0:
            time.sleep(remaining)
