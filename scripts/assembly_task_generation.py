#!/usr/bin/env python

import copy
import json
import math

config = json.load(open('./configs/ur3.json','r'))
SAFE_HEIGHT = config['safe_height']
GRASP_OFFSET = config['grasp_offset']
GRASP_EFFORT = config['grasp_effort']
RELEASE_EFFORT = config['release_effort']
NUM_ITERATIONS = config['num_iterations']
WORKSPACE_POSITION = config['workspace_position']
HOME_POSITION = config['home_position']
DOWN_GY_ORIENTATION = config['down_gy_orientation']
DOWN_GX_ORIENTATION = config['down_gx_orientation']
SPACING = config['block_spacing']

BLOCK_1x4 = (0.031,0.126,0.038)
BLOCK_1x3 = (0.031,0.095,0.058)
BLOCK_1x1 = (0.031,0.031,0.058)

class Queue:

    def __init__(self, origin_position, orientation, num_items, item_dimensions, spacing, name_unique='', offset_z=True):
        self.origin_position = origin_position
        self.number_of_items = num_items
        self.idims = item_dimensions
        self.spacing = spacing
        self._index = 0
        self.name = 'queue' + str(item_dimensions[0])\
                    + 'x' + str(item_dimensions[1])\
                    + 'x' + str(item_dimensions[2]) + name_unique
        self.offset_z = offset_z

        if orientation == 'HORIZONTAL_LEFT' or orientation == 'HORIZONTAL_RIGHT':
            self.orientation = orientation
        else:
            raise Exception('Invalid Orientation Enum');

    def _get(self,index):
        global DOWN_GY_ORIENTATION

        target_position = {
            'x': self.origin_position['x']
                + (index * (self.spacing + self.idims[0]) + 0.5 * self.idims[0])
                * (-1 if self.orientation == "HORIZONTAL_LEFT" else 1),
            'y': self.origin_position['y'] + 0.5 * self.idims[1],
            'z': self.origin_position['z'] + self.idims[2] * (0.5 if self.offset_z else 1)
        }

        target_orientation = copy.deepcopy(DOWN_GY_ORIENTATION)

        return target_position, target_orientation

    def get_next(self):
        global SAFE_HEIGHT, GRASP_OFFSET, GRASP_EFFORT

        obj_id =  self.name + '_' + str(self._index)

        if self._index >= self.number_of_items:
            raise Exception('There are no more items in this queue')

        target_position, target_orientation = self._get(self._index)

        task_list = [
            # move from current position to above queue item
            {
                "name": "move",
                "position": {
                    'x': target_position['x'],
                    'y': target_position['y'],
                    'z': SAFE_HEIGHT
                },
                "orientation": target_orientation
            },
            # move down to item
            {
                "name": "move",
                "position": {
                    'x': target_position['x'],
                    'y': target_position['y'],
                    'z': target_position['z'] + GRASP_OFFSET
                },
                "orientation": target_orientation
            },
            # grasp item
            {
                "name": "grasp",
                "effort": GRASP_EFFORT
            },
            # raise to homing position
            {
                "name": "move",
                "position": {
                    'x': target_position['x'],
                    'y': target_position['y'],
                    'z': SAFE_HEIGHT
                },
                "orientation": target_orientation
            }
        ]

        self._index += 1
        return task_list, obj_id

    def env_list(self):

        obj_list = []

        for index in range(0,self.number_of_items):
            target_position, target_orientation = self._get(index)
            target_position['z'] = target_position['z'] * (1 if self.offset_z else 0.5)

            obj_list.append({
                'name': self.name + '_' + str(index),
                'representation': 'box',
                'position': target_position,
                'orientation': {
                    'x': 0,
                    'y': 0,
                    'z': 0,
                    'w': 1
                },
                'size': {
                    'x': (self.idims[0]),
                    'y': (self.idims[1]),
                    'z': (self.idims[2])
                }
            })

        return obj_list

class AssemblyTask:

    def __init__(self):
        #TODO perhaps move the queues here, set all functions except generate to
        # static
        pass

    def home_position(self):
        return {
            'name': 'move',
            'position': copy.deepcopy(HOME_POSITION),
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        }

    def wait_for_human(self):
        return {
            'name': 'wait',
            'condition': 'button'
        }

    def build_base(self,queue_b4x1,queue_b3x1):
        task_list = []
        task_list.append({
            'name': 'logger',
            'msg': 'Task Progress: Building Base'
        })

        li, id = queue_b3x1.get_next()
        task_list = task_list + li

        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x3[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x3[1],
                'z': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x3[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x3[1],
                'z': WORKSPACE_POSITION['z'] + BLOCK_1x3[2] * 0.5 + GRASP_OFFSET
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release',
            'effort': RELEASE_EFFORT
        })

        li, id = queue_b3x1.get_next()
        task_list = task_list + li

        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x3[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x3[1],
                'z': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x3[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x3[1],
                'z': WORKSPACE_POSITION['z'] + BLOCK_1x3[2] * 0.5 + GRASP_OFFSET
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release',
            'effort': RELEASE_EFFORT
        })

        li, id = queue_b4x1.get_next()
        task_list = task_list + li

        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x4[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x4[1],
                'z': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
            },
            'orientation': copy.deepcopy(DOWN_GX_ORIENTATION)
        })
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x4[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x4[1],
                'z': WORKSPACE_POSITION['z'] + BLOCK_1x4[2] * 0.5 + GRASP_OFFSET
            },
            'orientation': copy.deepcopy(DOWN_GX_ORIENTATION)
        })
        task_list.append({
            'name': 'release',
            'effort': RELEASE_EFFORT
        })

        li, id = queue_b4x1.get_next()
        task_list = task_list + li

        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x4[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x4[1],
                'z': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
            },
            'orientation': copy.deepcopy(DOWN_GX_ORIENTATION)
        })
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x4[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x4[1],
                'z': WORKSPACE_POSITION['z'] + BLOCK_1x4[2] * 0.5 + GRASP_OFFSET
            },
            'orientation': copy.deepcopy(DOWN_GX_ORIENTATION)
        })
        task_list.append({
            'name': 'release',
            'effort': RELEASE_EFFORT
        })

        return task_list

    def build_pillars(self,queue_b1x1):
        task_list = []

        task_list.append({
            'name': 'logger',
            'msg': '\nTask Progress: Building Pillars Layer\n'
        })

        li, id = queue_b1x1.get_next()
        task_list = task_list + li

        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x1[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x1[1],
                'z': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x1[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x1[1],
                'z': WORKSPACE_POSITION['z'] + BLOCK_1x1[2] * 0.5 + GRASP_OFFSET
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release',
            'effort': RELEASE_EFFORT
        })

        li, id = queue_b1x1.get_next()
        task_list = task_list + li

        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x1[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x1[1],
                'z': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x1[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x1[1],
                'z': WORKSPACE_POSITION['z'] + BLOCK_1x1[2] * 0.5 + GRASP_OFFSET
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release',
            'effort': RELEASE_EFFORT
        })

        li, id = queue_b1x1.get_next()
        task_list = task_list + li

        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x1[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x1[1],
                'z': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x1[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x1[1],
                'z': WORKSPACE_POSITION['z'] + BLOCK_1x1[2] * 0.5 + GRASP_OFFSET
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release',
            'effort': RELEASE_EFFORT
        })

        li, id = queue_b1x1.get_next()
        task_list = task_list + li

        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x1[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x1[1],
                'z': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x1[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x1[1],
                'z': WORKSPACE_POSITION['z'] + BLOCK_1x1[2] * 0.5 + GRASP_OFFSET
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release',
            'effort': RELEASE_EFFORT
        })

        return task_list

    def build_top(self,queue_b4x1,queue_b3x1):
        task_list = []

        task_list.append({
            'name': 'logger',
            'msg': 'Task Progress: Building Top'
        })

        li, id = queue_b4x1.get_next()
        task_list = task_list + li

        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x4[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x4[1],
                'z': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
            },
            'orientation': copy.deepcopy(DOWN_GX_ORIENTATION)
        })
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x4[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x4[1],
                'z': WORKSPACE_POSITION['z'] + BLOCK_1x4[2] * 0.5 + GRASP_OFFSET
            },
            'orientation': copy.deepcopy(DOWN_GX_ORIENTATION)
        })
        task_list.append({
            'name': 'release',
            'effort': RELEASE_EFFORT
        })

        li, id = queue_b4x1.get_next()
        task_list = task_list + li

        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x4[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x4[1],
                'z': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
            },
            'orientation': copy.deepcopy(DOWN_GX_ORIENTATION)
        })
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x4[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x4[1],
                'z': WORKSPACE_POSITION['z'] + BLOCK_1x4[2] * 0.5 + GRASP_OFFSET
            },
            'orientation': copy.deepcopy(DOWN_GX_ORIENTATION)
        })
        task_list.append({
            'name': 'release',
            'effort': RELEASE_EFFORT
        })

        li, id = queue_b3x1.get_next()
        task_list = task_list + li

        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x3[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x3[1],
                'z': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x3[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x3[1],
                'z': WORKSPACE_POSITION['z'] + BLOCK_1x3[2] * 0.5 + GRASP_OFFSET
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release',
            'effort': RELEASE_EFFORT
        })

        li, id = queue_b3x1.get_next()
        task_list = task_list + li

        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x3[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x3[1],
                'z': WORKSPACE_POSITION['z'] + SAFE_HEIGHT
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + BLOCK_1x3[0],
                'y': WORKSPACE_POSITION['y'] + BLOCK_1x3[1],
                'z': WORKSPACE_POSITION['z'] + BLOCK_1x3[2] * 0.5 + GRASP_OFFSET
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release',
            'effort': RELEASE_EFFORT
        })

        return task_list

    def generate(self,queue_b4x1,queue_b3x1,queue_b1x1_1,queue_b1x1_2):
        task_list = []

        task_list.append(self.home_position())

        task_list.append({
            'name': 'release',
            'effort': RELEASE_EFFORT
        })

        for i in range(0,NUM_ITERATIONS):
            task_list.append({
                'name': 'logger',
                'msg': 'Task Iteration = ' + str(i)
            })

            task_list += self.build_base(queue_b4x1,queue_b3x1)
            #task_list += self.build_pillars(queue_b1x1_1)
            #task_list += self.build_pillars(queue_b1x1_2)
            #task_list += self.build_top(queue_b4x1,queue_b3x1)
            #task_list.append(self.home_position())
            #task_list.append(self.wait_for_human())

        return task_list


if __name__ == "__main__":

    # origin_position, orientation, num_items, item_dimensions, spacing
    queue_b4x1 = Queue({'x':0.18,'y':0.2,'z':0},'HORIZONTAL_RIGHT',4,BLOCK_1x4,SPACING,offset_z=False)
    queue_b3x1 = Queue({'x':0.18,'y':0.35,'z':0},'HORIZONTAL_RIGHT',4,BLOCK_1x3,SPACING)
    queue_b1x1_1 = Queue({'x':-0.18,'y':0.25,'z':0},'HORIZONTAL_LEFT',4,BLOCK_1x1,SPACING,'_1')
    queue_b1x1_2 = Queue({'x':-0.18,'y':0.3,'z':0},'HORIZONTAL_LEFT',4,BLOCK_1x1,SPACING,'_2')

    taskGen = AssemblyTask()
    task_list = taskGen.generate(queue_b4x1,queue_b3x1,queue_b1x1_1,queue_b1x1_2)

    # convert to radians if Euler angles
    for t in task_list:
        if t['name'] == 'move' and 'w' not in t['orientation']:
            t['orientation']['x'] = t['orientation']['x'] / 180.0 * math.pi
            t['orientation']['y'] = t['orientation']['y'] / 180.0 * math.pi
            t['orientation']['z'] = t['orientation']['z'] / 180.0 * math.pi

    env_list = []
    env_list += queue_b4x1.env_list()
    env_list += queue_b3x1.env_list()
    env_list += queue_b1x1_1.env_list()
    env_list += queue_b1x1_2.env_list()

    task = {
        'task': task_list,
        'environment': env_list
    }

    # save final file
    f = open('../plans/test.json','w')
    json.dump(task,f,indent=4)
