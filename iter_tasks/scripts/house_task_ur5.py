#!/usr/bin/env python

import os
import json
import copy

import iter_tasks_tools.data_types as dt
import iter_tasks_tools.primitives as pm


SAFE_HEIGHT_OFFSET = 0.4

BLOCK_SMALL = (0.02032,0.05842,0.01524)
BLOCK_LARGE = (0.02032,0.10,0.01524)

DOWN_GX_ORIENTATION = dt.orientation(0.707,0,-0.707,0)
DOWN_GY_ORIENTATION = dt.orientation(0.5,0.5,-0.5,0.5)

HOME_POSITION = dt.position(0,0.35,0.2)
HOME_ORIENTATION = copy.deepcopy(DOWN_GX_ORIENTATION)

WORKSPACE_POSITION = dt.position(0.1,0.375,-0.17)
REGION_POSITION = dt.position(0,0.5,0.2)
REGION_ORIENTATION = copy.deepcopy(DOWN_GX_ORIENTATION)

GRASP_EFFORT = 0.66
RELEASE_EFFORT_REGION = 0.35
RELEASE_EFFORT_WORKSPACE = 0.57

GRASP_OFFSET = dt.pose(dt.position(0,0,0.16),REGION_ORIENTATION)


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

def pick_and_place_block(block_type,target_position,target_orientation):

    safe_position = copy.deepcopy(target_position)
    safe_position['z'] += SAFE_HEIGHT_OFFSET

    action = pm.pick_and_place_vision(
        object_type=block_type,
        path_to_region=[
            pm.move(REGION_POSITION,DOWN_GY_ORIENTATION)
        ],
        path_to_destination=[
            pm.move(REGION_POSITION,DOWN_GY_ORIENTATION),
            pm.move(safe_position,target_orientation),
            pm.move(target_position,target_orientation)
        ],
        grasp_effort=GRASP_EFFORT,
        release_effort=RELEASE_EFFORT_WORKSPACE,
        grasp_offset=GRASP_OFFSET)
    return action

def build_house_base():
    task_list = []

    task_list.append(pm.logger('Building House'))
    task_list.append(pm.release(RELEASE_EFFORT_REGION))

    position = copy.deepcopy(WORKSPACE_POSITION)
    position['x'] += 0.5 * BLOCK_LARGE[0] + GRASP_OFFSET['position']['x']
    position['y'] += 0.5 * BLOCK_LARGE[1] + GRASP_OFFSET['position']['y']
    position['z'] += 0.5 * BLOCK_LARGE[2] + GRASP_OFFSET['position']['z'] + 0.0005
    task_list.append(pick_and_place_block(
        block_type='large',
        target_position=copy.deepcopy(position),
        target_orientation=DOWN_GY_ORIENTATION))

    position = copy.deepcopy(WORKSPACE_POSITION)
    position['x'] += BLOCK_SMALL[1] - 0.5 * BLOCK_LARGE[0] + GRASP_OFFSET['position']['x']
    position['y'] += 0.5 * BLOCK_LARGE[1] + GRASP_OFFSET['position']['y']
    position['z'] += 0.5 * BLOCK_LARGE[2] + GRASP_OFFSET['position']['z'] + 0.0005
    task_list.append(pick_and_place_block(
        block_type='large',
        target_position=copy.deepcopy(position),
        target_orientation=DOWN_GY_ORIENTATION))

    position = copy.deepcopy(WORKSPACE_POSITION)
    position['x'] += BLOCK_SMALL[0] + GRASP_OFFSET['position']['x']
    position['y'] += 0.5 * BLOCK_LARGE[0] + GRASP_OFFSET['position']['y']
    position['z'] += 1.5 * BLOCK_SMALL[2] + GRASP_OFFSET['position']['z'] + 0.0005
    task_list.append(pick_and_place_block(
        block_type='small',
        target_position=copy.deepcopy(position),
        target_orientation=DOWN_GY_ORIENTATION))

    position = copy.deepcopy(WORKSPACE_POSITION)
    position['x'] += BLOCK_SMALL[0] + GRASP_OFFSET['position']['x']
    position['y'] += BLOCK_LARGE[1] - 0.5 * BLOCK_SMALL[0] + GRASP_OFFSET['position']['y']
    position['z'] += 1.5 * BLOCK_SMALL[2] + GRASP_OFFSET['position']['z'] + 0.0005
    task_list.append(pick_and_place_block(
        block_type='small',
        target_position=copy.deepcopy(position),
        target_orientation=DOWN_GY_ORIENTATION))

    position = copy.deepcopy(WORKSPACE_POSITION)
    position['x'] += 0.5 * BLOCK_LARGE[0] + GRASP_OFFSET['position']['x']
    position['y'] += 0.5 * BLOCK_LARGE[1] + GRASP_OFFSET['position']['y']
    position['z'] += 2.5 * BLOCK_LARGE[2] + GRASP_OFFSET['position']['z'] + 0.0005
    task_list.append(pick_and_place_block(
        block_type='large',
        target_position=copy.deepcopy(position),
        target_orientation=DOWN_GY_ORIENTATION))

    position = copy.deepcopy(WORKSPACE_POSITION)
    position['x'] += BLOCK_SMALL[1] - 0.5 * BLOCK_LARGE[0] + GRASP_OFFSET['position']['x']
    position['y'] += 0.5 * BLOCK_LARGE[1] + GRASP_OFFSET['position']['y']
    position['z'] += 2.5 * BLOCK_LARGE[2] + GRASP_OFFSET['position']['z'] + 0.0005
    task_list.append(pick_and_place_block(
        block_type='large',
        target_position=copy.deepcopy(position),
        target_orientation=DOWN_GY_ORIENTATION))

    return task_list

def static_environment():
    env_list = []

    env_list.append(dt.environment_object(
        name='tabletop',
        position=dt.position(0,0.5,-0.19),
        orientation=dt.orientation(0,0,0,1),
        size=dt.size(1,0.6,0.015)))

    return env_list


if __name__ == "__main__":
    task_list = []
    task_list += move_home()
    task_list += wait_for_human()
    task_list += build_house_base()
    task_list += move_home()
    task_list += wait_for_human()

    env_list = []
    env_list += static_environment()

    plan = dt.plan(
        title='UR5 Simple Vision Pick-And-Place Task',
        author='Curt Henrichs',
        description='Simple vision task to prove out vision pipeline',
        version='0.0.1',
        frame_id='base_link',
        environment=env_list,
        task=task_list)

    path_to_scripts = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(path_to_scripts,'../plans/simple_vision.json')
    f = open(file_path,'w')
    json.dump(plan,f,indent=2)
