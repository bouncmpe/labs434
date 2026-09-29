"""Simulate a model in the passive viewer and plot live sensor readings.

Usage:
    python examples/sensors/plot_sensors.py MODEL.xml SENSOR [SENSOR ...]

Examples:
    python examples/sensors/plot_sensors.py examples/sensors/touch_sensor.xml touch
    python examples/sensors/plot_sensors.py examples/sensors/imu_sensors.xml accel gyro
    python examples/sensors/plot_sensors.py examples/sensors/rangefinder_sensor.xml range0 range4

Every component of every listed sensor is drawn as a separate line. Use the viewer's
Control panel to drive actuators (e.g. the arm in force_torque_sensor.xml).
"""

import argparse
import time

import mujoco
import mujoco.viewer

HISTORY = 500  # number of samples shown in the plot

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("model", help="path to the MJCF model")
parser.add_argument("sensors", nargs="+", help="names of the sensors to plot")
args = parser.parse_args()

m = mujoco.MjModel.from_xml_path(args.model)
d = mujoco.MjData(m)

# One line per sensor component, e.g. accel[0], accel[1], accel[2].
lines = []  # (sensor name, component index)
for name in args.sensors:
    dim = m.sensor(name).dim[0]
    lines += [(name, i) for i in range(dim)]
if len(lines) > mujoco.mjMAXLINE:
    raise SystemExit(f"too many lines to plot ({len(lines)} > {mujoco.mjMAXLINE})")

fig = mujoco.MjvFigure()
mujoco.mjv_defaultFigure(fig)
fig.title = ", ".join(args.sensors)
fig.xlabel = "time (s)"
fig.xformat = "%.1f"
fig.yformat = "%.2f"
for k, (name, i) in enumerate(lines):
    fig.linename[k] = name if m.sensor(name).dim[0] == 1 else f"{name}[{i}]"
fig.flg_legend = 1
fig.flg_extend = 1
fig.figurergba[3] = 0.5  # semi-transparent background

samples = []  # (time, [value of each line])

with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        step_start = time.time()
        mujoco.mj_step(m, d)

        samples.append((d.time, [d.sensor(name).data[i] for name, i in lines]))
        samples = samples[-HISTORY:]

        # linedata holds x, y pairs: [x0, y0, x1, y1, ...].
        for k in range(len(lines)):
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

        viewer.sync()

        remaining = m.opt.timestep - (time.time() - step_start)
        if remaining > 0:
            time.sleep(remaining)
