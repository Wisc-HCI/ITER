# Olivia Zhao 6/10/19

import json

def follow_path(position, orientation, size):
    return json.dumps({'name': 'tabletop',
                    'position': position,
                    'orientation': orientation,
                    'size': size})

def static_object_grasp(position, orientation, size):
    return json.dumps({'name': 'tabletop',
                        'position': position,
                        'orientation': orientation,
                        'size': size})

def static_pick_and_place(position, orientation, size):
    return json.dumps({'name': 'test',
                        'position': position,
                        'orientation': orientation,
                        'size': size})
