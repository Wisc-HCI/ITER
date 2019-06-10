#!/usr/bin/env python

# note there is a difference between robot orientation and base_link orientation
import sys
import copy
import json
import math

BLOCK_SMALL = (0.02032,0.05842,0.01524)
BLOCK_LARGE = (0.02032,0.10,0.01524)

if len(sys.argv) != 2:
    print 'must supply the robot config file to use'
    exit()

configFileName = sys.argv[1]

config = json.load(open('./configs/house/'+ configFileName +'.json','r'))
SAFE_HEIGHT = config['safe_height']
GRASP_OFFSET = config['grasp_offset']
GRASP_EFFORT = config['grasp_effort']
RELEASE_EFFORT_QUEUE = config['release_effort_queue']
RELEASE_EFFORT_WORKSPACE = config['release_effort_workspace']
NUM_ITERATIONS = config['num_iterations']
WORKSPACE_POSITION = config['workspace_position']
HOME_POSITION = config['home_position']
DOWN_GY_ORIENTATION = config['down_gy_orientation']
DOWN_GX_ORIENTATION = config['down_gx_orientation']
SPACING = config['block_spacing']

USE_TABLE = False
if 'table' in config.keys():
    USE_TABLE = True
    TABLE = config['table']
WORKSPACE_ENV = config['workspace_env']

class VisionRegion:

    def __init__(self):
        pass

    def _get(self,index):
        return target_position, target_orientation

    def get_next(self):
        return task_list, obj_id

    def pick_n_place(self,target_position,target_orientation):
        task_list = []

        li, id = self.get_next()
        task_list = task_list + li
        task_list.append({
            'name': 'move',
            'position': {
                'x': target_position['x'],
                'y': target_position['y'],
                'z': target_position['safe']
            },
            'orientation': copy.deepcopy(target_orientation)
        })
        task_list.append({
            'name': 'move',
            'position': {
                'x': target_position['x'],
                'y': target_position['y'],
                'z': target_position['z']
            },
            'orientation': copy.deepcopy(target_orientation)
        })
        task_list.append({
            'name': 'release',
            'effort': RELEASE_EFFORT_WORKSPACE
        })
        task_list.append({
            "name": "disconnect_object",
            "object_name": id
        })
        task_list.append({
            'name': 'move',
            'position': {
                'x': target_position['x'],
                'y': target_position['y'],
                'z': target_position['safe']
            },
            'orientation': copy.deepcopy(target_orientation)
        })

        return task_list

    def env_list(self):
        return []


class AssemblyTask:

    def __init__(self, region):
        self._region = region

    def home_position(self):
        return {
            'name': 'move',
            'position': copy.deepcopy(HOME_POSITION),
            'orientation': copy.deepcopy(DOWN_GX_ORIENTATION)
        }

    def wait_for_human(self):
        return {
            'name': 'wait',
            'condition': 'button'
        }

    def build_house(self, ):
        task_list = []

        task_list.append({
            'name': 'logger',
            'msg': 'Task Progress: Building House 1'
        })

        task_list = task_list + queue_large.pick_n_place({
            'x': WORKSPACE_POSITION['x'] + 0.5 * BLOCK_LARGE[0],
            'y': WORKSPACE_POSITION['y'] + BLOCK_LARGE[1] * 0.5,
            'z': WORKSPACE_POSITION['z'] + BLOCK_LARGE[2] * 0.5 + GRASP_OFFSET + 0.005,
            'safe': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
        },DOWN_GY_ORIENTATION)

        task_list = task_list + queue_large.pick_n_place({
            'x': WORKSPACE_POSITION['x'] + BLOCK_SMALL[1] - 0.5 * BLOCK_LARGE[0],
            'y': WORKSPACE_POSITION['y'] + BLOCK_LARGE[1] * 0.5,
            'z': WORKSPACE_POSITION['z'] + BLOCK_LARGE[2] * 0.5 + GRASP_OFFSET + 0.0005,
            'safe': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
        },DOWN_GY_ORIENTATION)

        task_list = task_list + queue_small.pick_n_place({
            'x': WORKSPACE_POSITION['x'] + BLOCK_SMALL[0],
            'y': WORKSPACE_POSITION['y'] + 0.5 * BLOCK_LARGE[0],
            'z': WORKSPACE_POSITION['z'] + BLOCK_SMALL[2] * 1.5 + GRASP_OFFSET + 0.0005,
            'safe': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
        },DOWN_GX_ORIENTATION)

        task_list = task_list + queue_small.pick_n_place({
            'x': WORKSPACE_POSITION['x'] + BLOCK_SMALL[0],
            'y': WORKSPACE_POSITION['y'] + BLOCK_LARGE[1] - 0.5 * BLOCK_SMALL[0],
            'z': WORKSPACE_POSITION['z'] + BLOCK_SMALL[2] * 1.5 + GRASP_OFFSET + 0.0005,
            'safe': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
        },DOWN_GX_ORIENTATION)

        task_list = task_list + queue_large.pick_n_place({
            'x': WORKSPACE_POSITION['x'] + 0.5 * BLOCK_LARGE[0],
            'y': WORKSPACE_POSITION['y'] + BLOCK_LARGE[1] * 0.5,
            'z': WORKSPACE_POSITION['z'] + BLOCK_LARGE[2] * 2.5 + GRASP_OFFSET + 0.0005,
            'safe': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
        },DOWN_GY_ORIENTATION)

        task_list = task_list + queue_large.pick_n_place({
            'x': WORKSPACE_POSITION['x'] + BLOCK_SMALL[1] - 0.5 * BLOCK_LARGE[0],
            'y': WORKSPACE_POSITION['y'] + BLOCK_LARGE[1] * 0.5,
            'z': WORKSPACE_POSITION['z'] + BLOCK_LARGE[2] * 2.5 + GRASP_OFFSET + 0.0005,
            'safe': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
        },DOWN_GY_ORIENTATION)

        return task_list

    def generate(self):
        task_list = []

        task_list.append(self.home_position())

        task_list.append({
            'name': 'release',
            'effort': RELEASE_EFFORT_QUEUE
        })

        for i in range(0,NUM_ITERATIONS):
            task_list.append({
                'name': 'logger',
                'msg': 'Task Iteration = ' + str(i)
            })

            task_list += self.build_house_1()
            task_list.append(self.home_position())
            task_list.append(self.wait_for_human())

        return task_list

    def env_list(self):
        return []


if __name__ == "__main__":

    region = VisionRegion()

    taskGen = AssemblyTask(region)
    task_list = taskGen.generate()

    env_list = []
    env_list += region.env_list()
    env_list += taskGen.env_list()

    if USE_TABLE:
        env_list.append({
            'name': 'tabletop',
            'representation': 'box',
            'position': TABLE['position'],
            'orientation': {
                'x': 0,
                'y': 0,
                'z': 0,
                'w': 1
            },
            'size': TABLE['size']
        })

    for element in WORKSPACE_ENV:
        env_list.append({
            'name': element['name'],
            'representation': 'box',
            'position': element['position'],
            'orientation': {
                'x': 0,
                'y': 0,
                'z': 0,
                'w': 1
            },
            'size': element['size']
        })

    task = {
        'task': task_list,
        'environment': env_list
    }

    # save final file
    f = open('../plans/house.json','w')
    json.dump(task,f,indent=4)
