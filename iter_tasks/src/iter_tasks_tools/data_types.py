# Olivia Zhao 6/10/19

import json

def position(x,y,z):
    return {'x': x, 'y': y, 'z':z}

def orientation(x, y, z, w):
    return {'x': x, 'y': y, 'z': z, 'w': w}

def size(x, y, z):
    return {'x': x, 'y': y, 'z': z}

def color(r, g, b, a):
    # range 0-1 for parameters
    return {'r': r, 'g': g, 'b': b, 'a': a}

def environment_object(name, position, orientation, size, color=color(0,1,0,1)):
    return {'name': name,
           'position': position,
           'orientation': orientation,
           'size': size,
           'color': color}

def plan(title,author,version,frame_id,environment,task, description=''):
    return {'title': title,
           'author': author,
           'version': version,
           'frame_id': frame_id,
           'description': description,
           'environment': environment,
           'task': task}
