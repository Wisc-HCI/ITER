#!/usr/bin/env python

import os
import json
import copy

import iter_tasks_tools.data_types as dt
import iter_tasks_tools.primitives as pm


task_case = "pooled"
build_house_1 = True
house_1_allocation = {"robot_base":True,"robot_mid_1":True,"robot_mid_2":True,"robot_roof":True}
build_house_2 = False
house_2_allocation = {"robot_base":False,"robot_mid_1":False,"robot_mid_2":False,"robot_roof":False}

SAFE_HEIGHT_OFFSET = 0.1

BLOCK_SMALL = (0.02032,0.05842,0.01524)
BLOCK_LARGE = (0.02032,0.10,0.01524)

DOWN_GX_ORIENTATION = dt.orientation(0.707,0,-0.707,0)
DOWN_GY_ORIENTATION = dt.orientation(0.5,0.5,-0.5,0.5)

HOME_POSITION = dt.position(0,0.25,0.3)
HOME_ORIENTATION = copy.deepcopy(DOWN_GX_ORIENTATION)

WAIT_POSITION = dt.position(0,0.25,0.3)
WAIT_ORIENTATION = dt.orientation(0,0,0,1)

WORKSPACE_POSITION = dt.position(0.2,0.25,-0.02)
REGION_POSITION = dt.position(-0.2,0.45,0.25)
REGION_ORIENTATION = copy.deepcopy(DOWN_GX_ORIENTATION)

GRASP_EFFORT = 0.59 #0.57
RELEASE_EFFORT_REGION = 0.25
RELEASE_EFFORT_WORKSPACE = 0.45

REGION_GRASP_OFFSET_LARGE = dt.pose(dt.position(0.005, 0.005, 0.135),
                                copy.deepcopy(REGION_ORIENTATION))

REGION_GRASP_OFFSET_SMALL = dt.pose(dt.position(0.005, 0.005, 0.135),
                                copy.deepcopy(REGION_ORIENTATION))

WORKSPACE_GRASP_OFFSET = dt.pose(dt.position(0,0,0.1675),
                                copy.deepcopy(REGION_ORIENTATION))


def move_home():
    task_list = []
    task_list.append(pm.logger('Moving to Home'))
    task_list.append(pm.move(HOME_POSITION,HOME_ORIENTATION))
    return task_list

def wait_for_human():
    task_list = []
    task_list.append(pm.logger('Waiting for Human'))
    task_list.append(pm.wait_button())
    return task_list

def pick_and_place_block(block_type,target_position,target_orientation,grasp_offset,vision_params):
    task_list = []

    safe_position = copy.deepcopy(target_position)
    safe_position['z'] += SAFE_HEIGHT_OFFSET

    task_list.append(pm.pick_and_place_vision(
        object_type=block_type,
        path_to_region=[],
        path_to_destination=[
            pm.move(safe_position,target_orientation),
            pm.move(target_position,target_orientation)
        ],
        grasp_effort=GRASP_EFFORT,
        release_effort=RELEASE_EFFORT_WORKSPACE,
        grasp_offset=grasp_offset,
        safe_height=SAFE_HEIGHT_OFFSET,
        vision_params=vision_params))

    task_list.append(pm.move(safe_position,target_orientation))
    task_list.append(pm.release(RELEASE_EFFORT_REGION))
    return task_list

def build_house(workplace_position, robot_base=True, robot_mid_1=True, robot_mid_2=True, robot_roof=True):
    task_list = []

    task_list.append(pm.logger('Building House Base'))
    task_list.append(pm.release(RELEASE_EFFORT_REGION))

    if robot_base:
        position = copy.deepcopy(workplace_position)
        position['x'] += 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.5 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 0.5 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            block_type='large',
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION,
            grasp_offset=REGION_GRASP_OFFSET_LARGE,
            vision_params={
                            "min_hue": 0,
                            "max_hue": 180
                          })

        position = copy.deepcopy(workplace_position)
        position['x'] += BLOCK_SMALL[1] - 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.5 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 0.5 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            block_type='large',
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION,
            grasp_offset=REGION_GRASP_OFFSET_LARGE,
            vision_params={
                            "min_hue": 0,
                            "max_hue": 180
                          })
    else:
        task_list += wait_for_human()

    if robot_mid_1:
        position = copy.deepcopy(workplace_position)
        position['x'] += BLOCK_SMALL[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 1.5 * BLOCK_SMALL[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            block_type='small',
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GX_ORIENTATION,
            grasp_offset=REGION_GRASP_OFFSET_SMALL,
            vision_params={
                            "min_hue": 0,
                            "max_hue": 180
                          })

        position = copy.deepcopy(workplace_position)
        position['x'] += BLOCK_SMALL[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += BLOCK_LARGE[1] - 0.5 * BLOCK_SMALL[0] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 1.5 * BLOCK_SMALL[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            block_type='small',
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GX_ORIENTATION,
            grasp_offset=REGION_GRASP_OFFSET_SMALL,
            vision_params={
                            "min_hue": 0,
                            "max_hue": 180
                          })
    else:
        task_list += wait_for_human()

    if robot_mid_2:
        position = copy.deepcopy(workplace_position)
        position['x'] += 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.5 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 2.5 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            block_type='large',
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION,
            grasp_offset=REGION_GRASP_OFFSET_LARGE,
            vision_params={
                            "min_hue": 0,
                            "max_hue": 180
                          })

        position = copy.deepcopy(workplace_position)
        position['x'] += BLOCK_SMALL[1] - 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.5 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 2.5 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            block_type='large',
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION,
            grasp_offset=REGION_GRASP_OFFSET_LARGE,
            vision_params={
                            "min_hue": 0,
                            "max_hue": 180
                          })
    else:
        task_list += wait_for_human()

    if robot_roof:
        # base
        position = copy.deepcopy(workplace_position)
        position['x'] += BLOCK_SMALL[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.5 * BLOCK_LARGE[1] + 0.1 * BLOCK_SMALL[0] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 3.5 * BLOCK_SMALL[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            block_type='small',
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GX_ORIENTATION,
            grasp_offset=REGION_GRASP_OFFSET_SMALL,
            vision_params={
                            "min_hue": 0,
                            "max_hue": 180
                          })

        # angles
        position = copy.deepcopy(workplace_position)
        position['x'] += 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.2 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 4 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            block_type='small',
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION,
            grasp_offset=REGION_GRASP_OFFSET_SMALL,
            vision_params={
                            "min_hue": 0,
                            "max_hue": 180
                          })

        position = copy.deepcopy(workplace_position)
        position['x'] += BLOCK_SMALL[1] - 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.2 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 4 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            block_type='small',
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION,
            grasp_offset=REGION_GRASP_OFFSET_SMALL,
            vision_params={
                            "min_hue": 0,
                            "max_hue": 180
                          })

        position = copy.deepcopy(workplace_position)
        position['x'] += 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.8 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 4 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            block_type='small',
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION,
            grasp_offset=REGION_GRASP_OFFSET_SMALL,
            vision_params={
                            "min_hue": 0,
                            "max_hue": 180
                          })

        position = copy.deepcopy(workplace_position)
        position['x'] += BLOCK_SMALL[1] - 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.8 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 4 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            block_type='small',
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION,
            grasp_offset=REGION_GRASP_OFFSET_SMALL,
            vision_params={
                            "min_hue": 0,
                            "max_hue": 180
                          })

    else:
        task_list += wait_for_human()

    return task_list

def static_environment():
    env_list = []

    env_list.append(dt.environment_object(
        name='tabletop',
        position=dt.position(0,0.35,-0.05),
        orientation=dt.orientation(0,0,0,1),
        size=dt.size(1,0.6,0.015)))

    return env_list


if __name__ == "__main__":

    task_list = []
    task_list += move_home()
    task_list += wait_for_human()

    if build_house_1:
        task_list += build_house(WORKSPACE_POSITION,**house_1_allocation)

    if build_house_2:
        task_list += move_home()
        task_list += wait_for_human()
        task_list += build_house(WORKSPACE_POSITION,**house_2_allocation)

    task_list += move_home()
    task_list += wait_for_human()

    env_list = []
    env_list += static_environment()

    plan = dt.plan(
        title='UR3e House Full Task',
        author='Curt Henrichs',
        description='Build house task',
        version='0.0.1',
        frame_id='base_link',
        environment=env_list,
        task=task_list)

    path_to_scripts = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(path_to_scripts,'../plans/ur3e/' + task_case + '.json')
    f = open(file_path,'w')
    json.dump(plan,f,indent=2)

    print 'Generated Plan!'
