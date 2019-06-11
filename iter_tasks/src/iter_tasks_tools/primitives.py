# Olivia Zhao 6/10/19


import json

def calibrate_robot_to_camera(path_to_region, tag_to_ee_transform, tag_id):
    return json.dumps({'name': 'calibrate_robot_to_camera',
                        'path_to_region':path_to_region,
                        'tag_to_ee_transform':tag_to_ee_transform,
                        'tag_id': tag_id })

def connect_object(object_name):
    return json.dumps({'name': 'connect_object',
                        'object_name': object_name})

def disconnect_object(test_id):
    return json.dumps({'name': 'disconnect_object',
                        'object_name': test_id})

def find_vision_object(object_type):
    return json.dumps({'name':'find_vision_object',
                        'object_type': object_type})

def get_pose():
    return json.dumps({'name':'get_pose'})

def grasp(effort):
    return json.dumps({'name':'grasp',
                        'effort': effort})

def logger(msg):
    return json.dumps({'name': 'logger',
                        'msg':msg})

def move(position, orientation):
    return json.dumps({'name': 'move',
                        'position': position,
                        'orientation': orientation})

def pick_and_place_static(path_to_object, path_to_destination,
    object_name, grasp_effort, release_effort):
    return json.dumps({'name':'pick_and_place_static',
                        'path_to_object': path_to_object,
                        'path_to_destination': path_to_destination,
                        'object_name': object_name,
                        'grasp_effort': grasp_effort,
                        'release_effort': release_effort
                        })

def pick_and_place_vision(object_type, path_to_region, path_to_destination,
    grasp_effort, release_effort):
    return json.dumps({'name': 'pick_and_place_vision',
                        'object_type': object_type,
                        'path_to_region': path_to_region,
                        'path_to_destination': path_to_destination,
                        'grasp_effort': grasp_effort,
                        'release_effort': release_effort
                        })

def release(effort):
    return json.dumps({'name': 'release',
                        'effort': effort
                        })

def wait_button(condition, timeout):
    return json.dumps({'name': 'wait',
                        'condition': condition,
                        'timeout': timeout
                        })

def wait_time(condition, value):
    return json.dumps({'name': 'wait',
                        'condition': condition,
                        'value': value
                        })
