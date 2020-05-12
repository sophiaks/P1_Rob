#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division, print_function

import cv2
import numpy as np
import math
from matplotlib import pyplot as plt
import time
from cv_bridge import CvBridge, CvBridgeError

lower = 0
upper = 1

cor_menor = np.array([230, 230, 230], dtype=np.uint8)
cor_maior = np.array([255, 255, 255], dtype=np.uint8)

#TypeError: Expected Ptr<cv::UMat> for argument '%s'


def line_intersecion(line1, line2):
        
    p1 = line1[0]
    p2 = line1[1]

    dy1 = p2[1] - p1[1]; 
    dx1 = p1[0] - p2[0]; 
    reta1 = dy1*(p1[0]) + dx1*(p1[1]);

    p3 = line2[0]
    p4 = line2[1]

    dy2 = p4[1] - p3[1]; 
    dx2 = p3[0] - p4[0]; 
    reta2 = dy2*(p3[0]) + dx2*(p3[1]);
    
    if dy1*dx2 - dy2*dx1 != 0:
        determinant = dy1*dx2 - dy2*dx1

    x = (dx2*reta1 - dx1*reta2)//determinant; 
    y = (dy1*reta2 - dy2*reta1)//determinant; 

    return (x, y)



def auto_canny(image, sigma=0.33):

    v = np.median(image)

    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    return edged
