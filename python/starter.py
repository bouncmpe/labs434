import time
import mujoco
import mujoco.viewer

def main():

    # Uncomment to start with an empty model
    # scene_spec = mujoco.MjSpec() 

    # Load existing XML models
    scene_spec = mujoco.MjSpec.from_file("scenes/empty_floor.xml")
    robot_spec = mujoco.MjSpec.from_file("models/mushr_car/model.xml")

    # Add the robot to the scene
    # A prefix is required to distinguish robots if we add more robots using the same model.
    scene_spec.attach(robot_spec, prefix="robot-", frame="world")

    # Initalize our simulation
    # Roughly, m keeps static (model) information, and d keeps dynamic (state) information. 
    m = scene_spec.compile()
    d = mujoco.MjData(m)


    # Helper construsts for the viewer for pause/unpause functionality.
    paused = False

    # Pressing SPACE key toggles the paused state. 
    # You can define other keys for other actions here.
    def key_callback(keycode):
      if chr(keycode)== ' ':
        global paused
        paused = not paused

    with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:

      # These actuator names are defined in the model XML file for the robot.
      # And we prefixed them to distinguish from other objects at the attachment.
      velocity = d.actuator("robot-throttle_velocity")
      steering = d.actuator("robot-steering")

      # Close the viewer automatically after 30 wall-clock-seconds.
      start = time.time()
      while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()

        if not paused:
            velocity.ctrl = 1.0 # update velocity control value
            steering.ctrl = 8.0 # update steering control value

            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            mujoco.mj_step(m, d)

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
          time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()
