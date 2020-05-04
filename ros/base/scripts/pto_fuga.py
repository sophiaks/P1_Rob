#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division, print_function

import cv2
import numpy as np
import math
from matplotlib import pyplot as plt
import time
from cv_bridge import CvBridge, CvBridgeError

cor_menor = np.array([240, 240, 240], dtype=np.uint8)
cor_maior = np.array([255, 255, 255], dtype=np.uint8)

#TypeError: Expected Ptr<cv::UMat> for argument '%s'



def func_pto(frame):
    '''Use esta função para encontrar o ponto de fuga'''
    while (True):    #     # Capture frame-by-fra
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(frame_hsv, cor_menor, cor_maior)
        blur = cv2.GaussianBlur(mask_white, (5, 5), 0)
        edges = cv2.Canny(blur ,50, 150, apertureSize = 3)
        lines = cv2.HoughLines(edges, 1, np.pi/180, 150)

        lista_m = []
        lista_h = []

        linhas_d_m = []
        linhas_d_x1 = []
        linhas_d_x2 = []
        linhas_d_y1 = []
        linhas_d_y2 = []
        linhas_d_h = []

        linhas_e_m = []
        linhas_e_x1 = []
        linhas_e_x2 = []
        linhas_e_y1 = []
        linhas_e_y2 = []
        linhas_e_h = []

        xis = []
        yis = []
        if lines != None:
            for x in range(0, len(lines)):    
                for rho, theta in lines[x]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(y0 - 1000*(a))
                    m = (y2 - y1)/(x2 - x1)
                    
                    h = y1 - m*x1

                    lista_h.append(h)
                    lista_m.append(m)

                    #direita
                    if m>0.3 and m<2:
                        linhas_d_m.append(m)
                        linhas_d_x1.append(x1)
                        linhas_d_x2.append(x2)
                        linhas_d_y1.append(y1)
                        linhas_d_y2.append(y2)
                        linhas_d_h.append(h)


                    #esquerda
                    elif m<-0.2 and m>-2:
                        linhas_e_m.append(m)
                        linhas_e_x1.append(x1)
                        linhas_e_x2.append(x2)
                        linhas_e_y1.append(y1)
                        linhas_e_y2.append(y2)
                        linhas_e_h.append(h)
                    
                    else:
                        lista_m.remove(m) 
                        lista_h.remove(h)


                    
                
            if len(lista_m) > 1 and lista_m[0] != lista_m[1]:
                x_i = (lista_h[1] - lista_h[0])/(lista_m[0] - lista_m[1])
                y_i = lista_m[0] * x_i + lista_h[0]
                x_i = int(x_i)
                y_i = int(y_i)
                xis.append(x_i)
                yis.append(y_i)


            x1 = 0
            x2 = 0
            x3 = 0
            x4 = 0
            y1 = 0
            y2 = 0
            y3 = 0
            y4 = 0
            
            #linha direita
            if len(linhas_d_m)>1:
                x1 = int(np.mean(linhas_d_x1))
                x2 = int(np.mean(linhas_d_x2))
                y1 = int(np.mean(linhas_d_y1))
                y2 = int(np.mean(linhas_d_y2))
                cv2.line(frame,(x1,y1), (x2,y2), (50,0,255),2) 
            
            #linha esquerda
            if len(linhas_e_m)>1:
                x3 = int(np.mean(linhas_e_x1))
                x4 = int(np.mean(linhas_e_x2))
                y3 = int(np.mean(linhas_e_y1))
                y4 = int(np.mean(linhas_e_y2))
                cv2.line(frame,(x3,y3), (x4,y4), (50,0,255),2) 

            #ponto de intersecção
            if x1!=0 and x2!=0 and x3!=0 and x4!=0:
                px = int(((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4))/((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)))
                py = int(((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-x4*y3))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)))
                cv2.circle(frame, (px, py), 1, (0,255,0), 5)
                pto = (px, py)

    # ver posicao 0, colocar em variavel, acessar variavel
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    return pto, linhas_e_m, linhas_d_m
