"""Draw every rangefinder beam and plot its reading in the same color.

Usage:
    python examples/sensors/draw_rangefinder_beams.py [MODEL.xml]

Each beam gets its own color, shared with its line in the plot. A beam that hits a geom
is drawn solid up to the hit point. A beam that reads -1 (nothing within the cutoff) is
drawn faded at full cutoff length, and its plot line drops to -1. The viewer's built-in
rangefinder drawing is turned off because it only shows beams that hit.
"""

import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

MODEL = sys.argv[1] if len(sys.argv) > 1 else "examples/sensors/rangefinder_sensor.xml"
COLORS = np.array([  # one RGB color per beam, reused cyclically
    [0.12, 0.47, 0.71],
    [1.00, 0.50, 0.05],
    [0.17, 0.63, 0.17],
    [0.84, 0.15, 0.16],
    [0.58, 0.40, 0.74],
    [0.55, 0.34, 0.29],
    [0.89, 0.47, 0.76],
    [0.74, 0.74, 0.13],
])
MISS_ALPHA = 0.25  # opacity of beams that hit nothing
MISS_LENGTH = 1.0  # beam length (m) drawn for misses when a sensor has no cutoff
HIT_WIDTH, MISS_WIDTH = 4, 2  # pixels
SPIN_SPEED = 1.0  # rad/s, initial scanner command; change it with the Control panel
HISTORY = 500  # number of samples shown in the plot

m = mujoco.MjModel.from_xml_path(MODEL)
d = mujoco.MjData(m)

rangefinders = [i for i in range(m.nsensor) if m.sensor_type[i] == mujoco.mjtSensor.mjSENS_RANGEFINDER]
colors = [COLORS[k % len(COLORS)] for k in range(len(rangefinders))]
if m.nu > 0:
    d.ctrl[0] = SPIN_SPEED


def draw_beams(scn):
    scn.ngeom = 0
    for i, rgb in zip(rangefinders, colors):
        site = m.sensor_objid[i]
        start = d.site_xpos[site]
        direction = d.site_xmat[site].reshape(3, 3)[:, 2]  # rays are cast along the site's +z axis
        distance = d.sensordata[m.sensor_adr[i]]

        if distance >= 0:
            length, alpha, width = distance, 1.0, HIT_WIDTH
        else:
            length, alpha, width = (m.sensor_cutoff[i] or MISS_LENGTH), MISS_ALPHA, MISS_WIDTH

        if scn.ngeom >= scn.maxgeom:
            return
        geom = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3), np.zeros(3), np.zeros(9),
                            np.append(rgb, alpha))
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, width, start, start + length * direction)
        scn.ngeom += 1


# Figure setup: one line per beam, in the beam's color.
fig = mujoco.MjvFigure()
mujoco.mjv_defaultFigure(fig)
fig.title = "rangefinder distance (m), -1 = no hit"
fig.xlabel = "time (s)"
fig.xformat = "%.1f"
fig.yformat = "%.1f"
for k, (i, rgb) in enumerate(zip(rangefinders, colors)):
    fig.linename[k] = m.sensor(i).name
    fig.linergb[k] = rgb
fig.flg_legend = 1
fig.flg_extend = 1
fig.figurergba[3] = 0.5  # semi-transparent background

samples = []  # (time, [reading of each beam])

with mujoco.viewer.launch_passive(m, d) as viewer:
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 0

    while viewer.is_running():
        step_start = time.time()
        mujoco.mj_step(m, d)

        samples.append((d.time, [d.sensordata[m.sensor_adr[i]] for i in rangefinders]))
        samples = samples[-HISTORY:]

        # linedata holds x, y pairs: [x0, y0, x1, y1, ...].
        for k in range(len(rangefinders)):
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
            draw_beams(viewer.user_scn)
        viewer.sync()

        remaining = m.opt.timestep - (time.time() - step_start)
        if remaining > 0:
            time.sleep(remaining)
