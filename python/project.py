import time
import mujoco
import mujoco.viewer
import random
import numpy as np
import scipy as sp

import cmpe434_dungeon as dungeon

# Helper construsts for the viewer for pause/unpause functionality.
paused = False

# Pressing SPACE key toggles the paused state.
def mujoco_viewer_callback(keycode):
    global paused
    if keycode == ord(' '):  # Use ord(' ') for space key comparison
        paused = not paused

def main():

    # Uncomment to start with an empty model
    # scene_spec = mujoco.MjSpec() 

    # Load existing XML models
    scene_spec = mujoco.MjSpec.from_file("scenes/empty_floor.xml")

    tiles, rooms, connections = dungeon.generate(3, 2, 8)
    for index, r in enumerate(rooms):
        (xmin, ymin, xmax, ymax) = dungeon.find_room_corners(r)
        scene_spec.worldbody.add_geom(name='R{}'.format(index), type=mujoco.mjtGeom.mjGEOM_PLANE, size=[(xmax-xmin)+1, (ymax-ymin)+1, 0.1], rgba=[0.8, 0.6, 0.4, 1],  pos=[(xmin+xmax), (ymin+ymax), 0])

    for pos, tile in tiles.items():
        if tile == "#":
            scene_spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[1, 1, 0.1], rgba=[0.8, 0.6, 0.4, 1],  pos=[pos[0]*2, pos[1]*2, 0])

    start_pos = random.choice([key for key in tiles.keys() if tiles[key] == "."])
    final_pos = random.choice([key for key in tiles.keys() if tiles[key] == "." and key != start_pos])

    scene_spec.worldbody.add_site(name='start', type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.5, 0.5, 0.01], rgba=[0, 0, 1, 1],  pos=[start_pos[0]*2, start_pos[1]*2, 0])
    scene_spec.worldbody.add_site(name='finish', type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.5, 0.5, 0.01], rgba=[1, 0, 0, 1],  pos=[final_pos[0]*2, final_pos[1]*2, 0])


    robot_spec = mujoco.MjSpec.from_file("models/mushr_car/model.xml")

    # Add robots to the scene:
    # - There must be a frame or site in the scene model to attach the robot to.
    # - A prefix is required if we add multiple robots using the same model.
    scene_spec.attach(robot_spec, frame="world", prefix="robot-")
    scene_spec.body("robot-buddy").pos[0] = start_pos[0] * 2
    scene_spec.body("robot-buddy").pos[1] = start_pos[1] * 2

    # Randomize initial orientation
    yaw = np.random.uniform(-np.pi, np.pi)
    euler = np.array([0.0, 0.0, yaw], dtype=np.float64)
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_euler2Quat(quat, euler, 'xyz')
    scene_spec.body("robot-buddy").quat[:] = quat

    # Add obstacles to the scene
    for i, room in enumerate(rooms):
        obs_pos = random.choice([tile for tile in room if tile != start_pos and tile != final_pos])
        scene_spec.worldbody.add_geom(
            name='Z{}'.format(i), 
            type=mujoco.mjtGeom.mjGEOM_CYLINDER, 
            size=[0.2, 0.05, 0.1], 
            rgba=[0.8, 0.0, 0.1, 1],  
            pos=[obs_pos[0]*2, obs_pos[1]*2, 0.08]
        )

    # Initalize our simulation
    # Roughly, m keeps static (model) information, and d keeps dynamic (state) information. 
    m = scene_spec.compile()
    d = mujoco.MjData(m)

    obstacles = [m.geom(i).id for i in range(m.ngeom) if m.geom(i).name.startswith("Z")]
    uniform_direction_dist = sp.stats.uniform_direction(2)
    obstacle_direction = [[x, y, 0] for x,y in uniform_direction_dist.rvs(len(obstacles))]
    unused = np.zeros(1, dtype=np.int32)

    with mujoco.viewer.launch_passive(m, d, key_callback=mujoco_viewer_callback) as viewer:

      viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
      viewer.cam.fixedcamid = m.camera("robot-third_person").id

      # These actuator names are defined in the model XML file for the robot.
      # Prefixes distinguish from other actuators from the same model.
      velocity = d.actuator("robot-throttle_velocity")
      steering = d.actuator("robot-steering")

      # Close the viewer automatically after 30 wall-clock-seconds.
      start = time.time()
      while viewer.is_running() and time.time() - start < 3000:
        step_start = time.time()

        if not paused:
            velocity.ctrl = 0.0 # update velocity control value
            steering.ctrl = 0.0 # update steering control value

            # Update obstables (bouncing movement)
            for i, x in enumerate(obstacles):
                dx = obstacle_direction[i][0]
                dy = obstacle_direction[i][1]

                px = m.geom_pos[x][0]
                py = m.geom_pos[x][1]
                pz = 0.02

                nearest_dist = mujoco.mj_ray(m, d, [px, py, pz], obstacle_direction[i], None, 1, -1, unused)

                if nearest_dist >= 0 and nearest_dist < 0.4:
                    obstacle_direction[i][0] = -dy
                    obstacle_direction[i][1] = dx

                m.geom_pos[x][0] = m.geom_pos[x][0]+dx*0.001
                m.geom_pos[x][1] = m.geom_pos[x][1]+dy*0.001

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
