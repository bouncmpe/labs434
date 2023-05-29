import mujoco
import mujoco_viewer
import numpy
import math
from scipy.spatial.transform import Rotation

model = mujoco.MjModel.from_xml_path('models/atlas_transpalet/scene.xml')
data = mujoco.MjData(model)

viewer = mujoco_viewer.MujocoViewer(model, data)
state = 0
state_end_time = 0

drive_speed_actuator = data.actuator("drive_speed")
steering_angle_actuator = data.actuator("steering_angle")
fork_height_actuator = data.actuator("fork_height")

robot_base_link = data.site("base_link")
palet_link = data.site("palet_link")
palet_drop = data.site("palet_drop")

wall_number = 12
wall_information = []

for wall_index in range(1, wall_number+1):
    wall = model.geom('wall_'+str(wall_index))
    wall_bottom_left_x = (wall.pos[0]-wall.size[0])
    wall_bottom_left_y = (wall.pos[1]-wall.size[1])
    wall_top_right_x = (wall.pos[0]+wall.size[0])
    wall_top_right_y = (wall.pos[1]+wall.size[1])

    wall_corners = [[wall_bottom_left_x,wall_bottom_left_y], [wall_top_right_x,wall_top_right_y]]
    wall_information.append(wall_corners)

dynamic_object_actuators = [data.actuator("dynamic_object_1"), data.actuator("dynamic_object_2"), data.actuator("dynamic_object_3")]
dynamic_object_positions = [data.site("dynamic_object_1_link"), data.site("dynamic_object_2_link"), data.site("dynamic_object_3_link")]
dynamic_object_speed = 0.2

def update_dynamic_objects(step_counter):
    for dynamic_object_index in range(0, len(dynamic_object_actuators)):
        dynamic_object_actuators[dynamic_object_index].ctrl = abs(math.sin(dynamic_object_speed * math.pi * step_counter / 1000)) * 100
        # print(dynamic_object_positions[dynamic_object_index].xpos)

step_counter = 0
for _ in range(4000):
    previous_time = data.time

    while((data.time - previous_time) < 1.0 / 60):
        update_dynamic_objects(step_counter)
        step_counter = step_counter + 1
        mujoco.mj_step(model, data)

    if viewer.is_alive:
        if(state == 0):
            forklift_rotation_matrix = numpy.array(robot_base_link.xmat).reshape(3,3)
            forklift_rotation =  Rotation.from_matrix(forklift_rotation_matrix)
            forklift_rotation_angles = forklift_rotation.as_euler("zyx",degrees=True)

            if(abs(forklift_rotation_angles[0] + 90) < 1):
                drive_speed_actuator.ctrl = 0.0
                steering_angle_actuator.ctrl = 0.0
                state = state + 1
                state_end_time = data.time
            else:
                drive_speed_actuator.ctrl = 1.0
                steering_angle_actuator.ctrl = -1.0

        elif(state == 1):
            drive_speed_actuator.ctrl = 0.0
            steering_angle_actuator.ctrl = 0.0

            if(abs(state_end_time - data.time) > 2):
                state = state + 1
                state_end_time = data.time

        elif(state == 2):
            forklift_position = robot_base_link.xpos
            palet_position = palet_link.xpos

            if(abs(forklift_position[1] - palet_position[1]) < 0.01):
                drive_speed_actuator.ctrl = 0.0
                steering_angle_actuator.ctrl = 0.0                    
                state = state + 1
            else:
                drive_speed_actuator.ctrl = -1 * abs(forklift_position[1] - palet_position[1])
                steering_angle_actuator.ctrl = 0.0

        elif(state == 3):
            forklift_rotation_matrix = numpy.array(robot_base_link.xmat).reshape(3,3)
            forklift_rotation =  Rotation.from_matrix(forklift_rotation_matrix)
            forklift_rotation_angles = forklift_rotation.as_euler("zyx",degrees=True)

            if(abs(forklift_rotation_angles[0] - 0) < 1):
                drive_speed_actuator.ctrl = 0.0
                steering_angle_actuator.ctrl = 0.0
                state = state + 1
                state_end_time = data.time
            else:
                drive_speed_actuator.ctrl = 1.0
                steering_angle_actuator.ctrl = 1.0

        elif(state == 4):
            forklift_position = robot_base_link.xpos
            palet_position = palet_link.xpos

            if(abs(forklift_position[0] - (palet_position[0] - 0.25)) < 0.1):
                drive_speed_actuator.ctrl = 0.0
                steering_angle_actuator.ctrl = 0.0
                state = state + 1
                state_end_time = data.time
                    
            else:
                drive_speed_actuator.ctrl = (-1 * abs(forklift_position[0] - (palet_position[0] - 0.25)) / 2)
                steering_angle_actuator.ctrl = 0.0

        elif(state == 5):
            drive_speed_actuator.ctrl = 0.0
            steering_angle_actuator.ctrl = 0.0

            if(abs(state_end_time - data.time) > 2):
                state = state + 1
                state_end_time = data.time
            else:
                fork_height_actuator.ctrl = abs(state_end_time - data.time)

        elif(state == 6):
            drive_speed_actuator.ctrl = 0.60
            steering_angle_actuator.ctrl = 0.8
            fork_height_actuator.ctrl = 1.0

        viewer.render()

    else:
        break

viewer.close()
