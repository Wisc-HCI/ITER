#!/usr/bin/env python

import copy
import json
import math

NUM_ITERATIONS = 1

WORKSPACE_POSITION = {
    'x': 0,
    'y': 0.4,
    'z': 0
}

SAFE_HEIGHT = 0.25
DOWN_GY_ORIENTATION = {
    'x': -0.265591116078,
    'y': 0.654797148311,
    'z': 0.271779272126,
    'w': 0.65332846323
}
DOWN_GX_ORIENTATION = {
    'x': -0.265591116078,
    'y': 0.654797148311,
    'z': 0.271779272126,
    'w': 0.65332846323
}

HOME_POSITION = {
    'x': 0,
    'y': 0.4,
    'z': SAFE_HEIGHT
}

UNIT_BLOCK = (0.025,0.025,0.05)
SPACING = .025

BLOCK_1x4 = (1,4,1)
BLOCK_1x3 = (1,3,1)
BLOCK_1x1 = (1,1,1)

class Queue:

    def __init__(self, origin_position, orientation, num_items, item_dimensions, spacing):
        self.origin_position = origin_position
        self.number_of_items = num_items
        self.idims = item_dimensions
        self.spacing = spacing
        self._index = 0

        if orientation == 'HORIZONTAL_LEFT' or orientation == 'HORIZONTAL_RIGHT':
            self.orientation = orientation
        else:
            raise Exception('Invalid Orientation Enum');

    def get_next(self):
        global SAFE_HEIGHT, DOWN_ORIENTATION, UNIT_BLOCK, GRASP_OFFSET

        if self._index >= self.number_of_items:
            raise Exception('There are no more items in this queue')

        target_position = {
            'x': self.origin_position['x']
                + (self._index * (self.spacing + self.idims[0] * UNIT_BLOCK[0]) + 0.5 * self.idims[0] * UNIT_BLOCK[0])
                * (-1 if self.orientation == "HORIZONTAL_LEFT" else 1),
            'y': self.origin_position['y'] + 0.5 * self.idims[1] * UNIT_BLOCK[1],
            'z': self.origin_position['z'] + self.idims[2] * UNIT_BLOCK[2] - 0.5 * UNIT_BLOCK[2]
        }

        target_orientation = copy.deepcopy(DOWN_GY_ORIENTATION)

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
                    'z': target_position['z']
                },
                "orientation": target_orientation
            },
            # grasp item
            {
                "name": "grasp",
                "effort": 1
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
        return task_list


if __name__ == "__main__":

    # origin_position, orientation, num_items, item_dimensions, spacing
    queue_b4x1 = Queue({'x':0.3,'y':0.2,'z':0},'HORIZONTAL_RIGHT',4,BLOCK_1x4,SPACING)
    queue_b3x1 = Queue({'x':0.3,'y':0.3,'z':0},'HORIZONTAL_RIGHT',4,BLOCK_1x3,SPACING)
    queue_b1x1_1 = Queue({'x':-0.3,'y':0.2,'z':0},'HORIZONTAL_LEFT',4,BLOCK_1x1,SPACING)
    queue_b1x1_2 = Queue({'x':-0.3,'y':0.3,'z':0},'HORIZONTAL_LEFT',4,BLOCK_1x1,SPACING)

    task_list = []

    # set home position
    task_list.append({
        'name': 'move',
        'position': copy.deepcopy(HOME_POSITION),
        'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
    })

    for i in range(0,NUM_ITERATIONS):
        # build base
        task_list = task_list + queue_b3x1.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 0.5 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 1.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 1 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })
        task_list = task_list + queue_b3x1.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 3.5 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 1.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 1 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })
        task_list = task_list + queue_b4x1.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 2 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 0.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 2 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GX_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })
        task_list = task_list + queue_b4x1.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 2 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 2.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 2 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GX_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })

        # build pillars
        task_list = task_list + queue_b1x1_1.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 0.5 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 0.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 3 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })
        task_list = task_list + queue_b1x1_1.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 3.5 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 0.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 3 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })
        task_list = task_list + queue_b1x1_1.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 3.5 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 2.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 3 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })
        task_list = task_list + queue_b1x1_1.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 0.5 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 2.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 3 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })

        task_list = task_list + queue_b1x1_2.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 0.5 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 0.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 4 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })
        task_list = task_list + queue_b1x1_2.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 3.5 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 0.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 4 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })
        task_list = task_list + queue_b1x1_2.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 3.5 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 2.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 4 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })
        task_list = task_list + queue_b1x1_2.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 0.5 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 2.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 4 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })

        # build top
        task_list = task_list + queue_b4x1.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 2 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 0.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 5 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GX_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })
        task_list = task_list + queue_b4x1.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 2 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 2.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 5 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GX_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })
        task_list = task_list + queue_b3x1.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 0.5 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 1.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 6 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })
        task_list = task_list + queue_b3x1.get_next()
        task_list.append({
            'name': 'move',
            'position': {
                'x': WORKSPACE_POSITION['x'] + 3.5 * UNIT_BLOCK[0],
                'y': WORKSPACE_POSITION['y'] + 1.5 * UNIT_BLOCK[1],
                'z': WORKSPACE_POSITION['z'] + 6 * UNIT_BLOCK[2]
            },
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })
        task_list.append({
            'name': 'release'
        })

        # home position
        task_list.append({
            'name': 'move',
            'position': copy.deepcopy(HOME_POSITION),
            'orientation': copy.deepcopy(DOWN_GY_ORIENTATION)
        })

        # wait for human
        task_list.append({
            'name': 'wait',
            'condition': 'button'
        })

    # convert to radians if Euler angles
    for t in task_list:
        if t['name'] == 'move' and 'w' not in t['orientation']:
            t['orientation']['x'] = t['orientation']['x'] / 180.0 * math.pi
            t['orientation']['y'] = t['orientation']['y'] / 180.0 * math.pi
            t['orientation']['z'] = t['orientation']['z'] / 180.0 * math.pi

    # save final file
    f = open('../plans/test.json','w')
    json.dump({'task':task_list},f,indent=4)
