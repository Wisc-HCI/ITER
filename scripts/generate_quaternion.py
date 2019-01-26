import math

# 90, 90, 90 is closest
# -180, 90, 0

x = (-180.0 / 180.0) * math.pi
y = (90.0 / 180.0) * math.pi
z = (0.0 / 180.0) * math.pi

c1 = math.cos(x/2)
s1 = math.sin(x/2)
c2 = math.cos(y/2)
s2 = math.sin(y/2)
c3 = math.cos(z/2)
s3 = math.sin(z/2)
c1c2 = c1 * c2
s1s2 = s1 * s2

quat = {
    'x': c1c2 * s3 + s1s2 * c3,
    'y': s1 * c2 * c3 + c1 * s2 * s3,
    'z': c1 * s2 * c3 - s1 * c2 * s3,
    'w': c1c2 * c3 - s1s2 * s3
}




'''
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
'''
