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

# cor_menor = np.array([10, 40, 60], dtype=np.uint8)
# cor_maior = np.array([ 27, 255, 255], dtype=np.uint8)

cor_menor = np.array([240, 240, 240], dtype=np.uint8)
cor_maior = np.array([255, 255, 255], dtype=np.uint8)

cor_menor2 = np.array([20, 100, 100], dtype=np.uint8)
cor_maior2 = np.array([30, 255, 255], dtype=np.uint8)

#TypeError: Expected Ptr<cv::UMat> for argument '%s'
def acha_ponto(frame):
    if frame is not None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        bordas = cv2.Canny(blur, 50, 150, apertureSize=3)
        bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)
        mask = cv2.inRange(bordas_color, cor_menor,cor_maior)
        lines = cv2.HoughLines(mask, 1, np.pi/180, 100)

        if lines is not None:
            print("Achou linha(s): {}".format(lines))
            for x in range(0, len(lines)):
                for rho, theta in lines[x]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 3000*(-b))
                    y1 = int(y0 + 3000*(a))
                    x2 = int(x0 - 3000*(-b))
                    y2 = int(y0 - 3000*(a))

                    if x2 != x1:
                        m = (y1-y0)/(x1-x0)

                    h = y0 - (m * x0)
                    p1 = (x1, y1)
                    p2 = (x2, y2)

                    if m < -0.3 and m > -19:
                        cv2.line(frame, (x1, y1),
                                    (x2, y2), (0, 255, 0), 1)
                        line1 = (p1, p2)
                        if line1 is not None:
                            print("Linha esquerda ok")

                    elif m > 0.3 and m < 15:
                        cv2.line(frame, (x1, y1),
                                    (x2, y2), (0, 255, 0), 1)
                        line2 = (p1, p2)
                        if line1 is not None:
                            print("Linha direita ok")

                    if line1 is not None and line2 is not None:
                        pi = line_intersecion(line1, line2)
                        ptos.append(pi)

        
            if len(ptos) > 0:
                if len(ptos) > 1:
                    ptom = np.array(ptos).mean(axis=0)
                else:
                    ptom = ptos[0]
                ptom = tuple(ptom)
    return ptom


def line_intersecion(line1, line2):
        
    p1 = line1[0]
    p2 = line1[1]

    dy1 = p2[1] - p1[1] 
    dx1 = p1[0] - p2[0]
    reta1 = dy1*(p1[0]) + dx1*(p1[1])

    p3 = line2[0]
    p4 = line2[1]
    dy2 = p4[1] - p3[1]; 
    dx2 = p3[0] - p4[0]; 
    reta2 = dy2*(p3[0]) + dx2*(p3[1])
    
    if dy1*dx2 - dy2*dx1 != 0:
        determinant = dy1*dx2 - dy2*dx1

    x = (dx2*reta1 - dx1*reta2)//determinant 
    y = (dy1*reta2 - dy2*reta1)//determinant 

    return (x, y)

def auto_canny(image, sigma=0.33):

    v = np.median(image)

    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    return edged




