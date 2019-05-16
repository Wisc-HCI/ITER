#!/usr/bin/env python
'''
Test the algorithm with an image before using it as a ros node.
'''

import os
import cv2
import time
import numpy as np

BIG_BLOCK_A = (22000,24000)
SML_BLOCK_A = (10000,12000)

image_np = cv2.imread(os.path.abspath("./data/test_img.JPG"))
height, width, channels = image_np.shape

hsv = cv2.cvtColor(image_np,cv2.COLOR_BGR2HSV)
thresh1 = cv2.inRange(hsv,(0,50,50),(179,255,255))

kernel = np.ones((5,5),np.uint8)
morphed = cv2.morphologyEx(thresh1,cv2.MORPH_OPEN,kernel)

_0, contours, _1 = cv2.findContours(morphed,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

final_img = image_np.copy()
for cnt in contours:
    # fit rectangle to object
    rect = cv2.minAreaRect(cnt)
    #rect[0] #.center (x,y) 'float'
    #rect[1] #.size (x,y) 'float'
    #rect[2] #.angle 'float'

    # filter by known attributes of blocks
    area = rect[1][0] * rect[1][1]
    print area
    if not ((area >= BIG_BLOCK_A[0] and area <= BIG_BLOCK_A[1]) or (area >= SML_BLOCK_A[0] and area <= SML_BLOCK_A[1])):
        continue

    # generate pose information
    {
        "position": {
            "x": rect[0][0] / width,
            "y": rect[0][1] / height,
            "z": 0
        },
        "orientation": {
            "x": 0,
            "y": 0,
            "z": rect[2]
        }
    }

    # update output image
    min = tuple([int(rect[0][i] - rect[1][i]/2) for i in range(0,2)])
    max = tuple([int(rect[0][i] + rect[1][i]/2) for i in range(0,2)])
    cv2.rectangle(final_img,min,max,(0,255,0),2)

cv2.imshow('Initial',image_np)
cv2.imshow('Final',final_img)

cv2.waitKey(1000 * 15)
cv2.destroyAllWindows()
