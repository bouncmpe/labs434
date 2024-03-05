import time

import mujoco
import mujoco.viewer

import cmpe434_utils

scene, scene_assets = cmpe434_utils.get_model('scenes/empty_floor.xml')
robot, robot_assets = cmpe434_utils.get_model('models/mushr_car/model.xml')
# robot, robot_assets = cmpe434_utils.get_model('models/mujoco_car/model.xml')
# robot, robot_assets = cmpe434_utils.get_model('models/skydio_x2/x2.xml')

# Add the robot to the scene.
scene.include_copy(robot)

# Combine all assets into a single dictionary.
all_assets = {**scene_assets, **robot_assets}

m = mujoco.MjModel.from_xml_string(scene.to_xml_string(), assets=all_assets)
d = mujoco.MjData(m)

paused = False # Global variable to control the pause state.

# Pressing SPACE key toggles the paused state. 
# You can define other keys for other actions here.
def key_callback(keycode):
  if chr(keycode) == ' ':
    global paused
    paused = not paused

with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:

  # velocity = m.actuator("throttle_velocity")
  # steering = m.actuator("steering")

  velocity = d.actuator("throttle_velocity")
  steering = d.actuator("steering")

  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()

    if not paused:
        velocity.ctrl = 1.0 # update velocity control value
        steering.ctrl = 1.0 # update steering control value

        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        mujoco.mj_step(m, d)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
