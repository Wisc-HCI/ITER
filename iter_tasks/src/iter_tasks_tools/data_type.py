# Olivia Zhao 6/10/19

import json

def position(x,y,z):
    return json.dumps({'x': x,
                        'y': y,
                        'z':z})

def orientation(y, x, z, w):
    return json.dumps({'y': y,
                        'x': x,
                        'z': z,
                        'w': w})

def size(y, x, z):
    return json.dumps({'y': y,
                        'x': x,
                        'z': z})
