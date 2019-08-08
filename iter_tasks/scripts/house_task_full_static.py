#!/usr/bin/env python

import os
import json
import copy

import iter_tasks_tools.data_types as dt
import iter_tasks_tools.primitives as pm

from iter_tasks_tools.queue import Queue, QueueSet


task_case = "pooled_static"
build_house_1 = True
house_1_allocation = {"robot_base":True,"robot_mid_1":True,"robot_mid_2":True,"robot_roof":True}
build_house_2 = False
house_2_allocation = {"robot_base":False,"robot_mid_1":False,"robot_mid_2":False,"robot_roof":False}

AFE_HEIGHT_OFFSET = 0.1

BLOCK_SMALL = (0.02032,0.05842,0.01524)
BLOCK_LARGE = (0.02032,0.10,0.01524)

DOWN_GX_ORIENTATION = dt.orientation(0.707,0,-0.707,0)
DOWN_GY_ORIENTATION = dt.orientation(0.5,0.5,-0.5,0.5)

HOME_POSITION = dt.position(0,0.25,0.3)
HOME_ORIENTATION = copy.deepcopy(DOWN_GX_ORIENTATION)

WAIT_POSITION = dt.position(0,0.25,0.3)
WAIT_ORIENTATION = dt.orientation(0,0,0,1)

WORKSPACE_POSITION = dt.position(0.2,0.25,-0.02)

GRASP_EFFORT = 0.59 #0.57
RELEASE_EFFORT_REGION = 0.25
RELEASE_EFFORT_WORKSPACE = 0.45

WORKSPACE_GRASP_OFFSET = dt.pose(dt.position(0,0,0.1675),
                                 copy.deepcopy(DOWN_GX_ORIENTATION))

obj_count = 0
dynamic_env_objs = []


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

def pick_and_place_block(queue, target_position, target_orientation):
    global obj_count, dynamic_env_objs

    task_list = []

    safe_target_position = copy.deepcopy(target_position)
    safe_target_position['z'] += SAFE_HEIGHT_OFFSET

    object_pose = queue.next()
    safe_object_position = copy.deepcopy(object_pose['position'])
    safe_object_position['z'] += SAFE_HEIGHT_OFFSET

    task_list.append(pm.pick_and_place_static(
        path_to_object=[
            pm.move(safe_object_position,object_pose['orientation']),
            pm.move(object_pose['position'],object_pose['orientation'])
        ],
        path_to_destination=[
            pm.move(safe_target_position,target_orientation),
            pm.move(target_position,target_orientation)
        ],
        object_name=obj_count,
        grasp_effort=GRASP_EFFORT,
        release_effort=RELEASE_EFFORT_WORKSPACE))

    obj_count += 1
    dynamic_env_objs.append(dt.environment_object(
        name='{}'.format(obj_count),
        position=object_pose['position'],
        orientation=object_pose['orientation'],
        size=dt.size(0.05,0.05,0.05)))

    task_list.append(pm.move(safe_target_position,target_orientation))
    task_list.append(pm.release(RELEASE_EFFORT_REGION))
    return task_list

def build_house(workplace_position, large_set, small_set, robot_base=True, robot_mid_1=True, robot_mid_2=True, robot_roof=True):
    task_list = []

    task_list.append(pm.logger('Building House Base'))
    task_list.append(pm.release(RELEASE_EFFORT_REGION))

    if robot_base:
        position = copy.deepcopy(workplace_position)
        position['x'] += 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.5 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 0.5 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            queue=large_set,
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION)

        position = copy.deepcopy(workplace_position)
        position['x'] += BLOCK_SMALL[1] - 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.5 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 0.5 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            queue=large_set,
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION)
    else:
        task_list += wait_for_human()

    if robot_mid_1:
        position = copy.deepcopy(workplace_position)
        position['x'] += BLOCK_SMALL[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 1.5 * BLOCK_SMALL[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            queue=small_set,
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GX_ORIENTATION)

        position = copy.deepcopy(workplace_position)
        position['x'] += BLOCK_SMALL[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += BLOCK_LARGE[1] - 0.5 * BLOCK_SMALL[0] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 1.5 * BLOCK_SMALL[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            queue=small_set,
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GX_ORIENTATION)
    else:
        task_list += wait_for_human()

    if robot_mid_2:
        position = copy.deepcopy(workplace_position)
        position['x'] += 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.5 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 2.5 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            queue=large_set,
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION)

        position = copy.deepcopy(workplace_position)
        position['x'] += BLOCK_SMALL[1] - 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.5 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 2.5 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            queue=large_set,
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION)
    else:
        task_list += wait_for_human()

    if robot_roof:
        # base
        position = copy.deepcopy(workplace_position)
        position['x'] += BLOCK_SMALL[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.5 * BLOCK_LARGE[1] + 0.1 * BLOCK_SMALL[0] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 3.5 * BLOCK_SMALL[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            queue=small_set,
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GX_ORIENTATION)

        # angles
        position = copy.deepcopy(workplace_position)
        position['x'] += 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.2 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 4 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            queue=small_set,
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION)

        position = copy.deepcopy(workplace_position)
        position['x'] += BLOCK_SMALL[1] - 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.2 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 4 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            queue=small_set,
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION)

        position = copy.deepcopy(workplace_position)
        position['x'] += 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.8 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 4 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            queue=small_set,
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION)

        position = copy.deepcopy(workplace_position)
        position['x'] += BLOCK_SMALL[1] - 0.5 * BLOCK_LARGE[0] + WORKSPACE_GRASP_OFFSET['position']['x']
        position['y'] += 0.8 * BLOCK_LARGE[1] + WORKSPACE_GRASP_OFFSET['position']['y']
        position['z'] += 4 * BLOCK_LARGE[2] + WORKSPACE_GRASP_OFFSET['position']['z'] + 0.0005
        task_list += pick_and_place_block(
            queue=small_set,
            target_position=copy.deepcopy(position),
            target_orientation=DOWN_GY_ORIENTATION)

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

    large_set = Queue(
        start_pose=dt.position(0,0,0),
        end_pose=dt.position(0,0,0),
        num_items = 4,
        orientation = DOWN_GX_ORIENTATION)

    small_set = QueueSet([
        Queue(
            start_pose=dt.position(0,0,0),
            end_pose=dt.position(0,0,0),
            num_items = 4,
            orientation = DOWN_GX_ORIENTATION),
        Queue(
            start_pose=dt.position(0,0,0),
            end_pose=dt.position(0,0,0),
            num_items = 4,
            orientation = DOWN_GX_ORIENTATION)
    ])

    task_list = []
    task_list += move_home()
    task_list += wait_for_human()

    if build_house_1:
        task_list += build_house(WORKSPACE_POSITION,large_set,small_set,**house_1_allocation)

    if build_house_2:
        task_list += move_home()
        task_list += wait_for_human()
        task_list += build_house(WORKSPACE_POSITION,large_set,small_set,**house_2_allocation)

    task_list += move_home()
    task_list += wait_for_human()

    env_list = []
    env_list += static_environment()
    env_list += dynamic_env_objs

    plan = dt.plan(
        title='UR3e House Full Task - Static',
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
