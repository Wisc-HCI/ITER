# Olivia Zhao 6/10/19


def calibrate_robot_to_camera(path_to_region, tag_to_ee_transform, tag_id):
    return {'name': 'calibrate_robot_to_camera',
            'path_to_region':path_to_region,
            'tag_to_ee_transform':tag_to_ee_transform,
            'tag_id': tag_id }

def connect_object(object_name):
    return {'name': 'connect_object',
            'object_name': object_name}

def disconnect_object(test_id):
    return {'name': 'disconnect_object',
            'object_name': test_id}

def find_vision_object(object_type, vision_params):
    return {'name':'find_vision_object',
            'object_type': object_type,
            'vision_params': vision_params}

def get_pose():
    return {'name':'get_pose'}

def grasp(effort):
    return {'name':'grasp',
            'effort': effort}

def logger(msg):
    return {'name': 'logger',
            'msg':msg}

def move(position, orientation):
    return {'name': 'move',
            'position': position,
            'orientation': orientation}

def pick_and_place_static(path_to_object, path_to_destination, object_name, grasp_effort, release_effort):
    return {'name':'pick_and_place_static',
            'path_to_object': path_to_object,
            'path_to_destination': path_to_destination,
            'object_name': object_name,
            'grasp_effort': grasp_effort,
            'release_effort': release_effort}

def pick_and_place_vision(object_type, path_to_region, path_to_destination, grasp_effort, release_effort, grasp_offset, vision_params):
    return {'name': 'pick_and_place_vision',
            'object_type': object_type,
            'path_to_region': path_to_region,
            'path_to_destination': path_to_destination,
            'grasp_effort': grasp_effort,
            'release_effort': release_effort,
            'grasp_offset': grasp_offset,
            'vision_params': vision_params}

def release(effort):
    return {'name': 'release',
            'effort': effort}

def wait_button(timeout=None):
    msg = {'name': 'wait',
            'condition': 'button'}
    if timeout != None:
        msg['timeout'] = timeout
    return msg

def wait_time(value):
    return {'name': 'wait',
            'condition': 'time',
            'value': value}
