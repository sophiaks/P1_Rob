#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division, print_function


import cv2
import numpy as np

cor_menor = np.array([240, 240, 240], dtype=np.uint8)
cor_maior = np.array([255, 255, 255], dtype=np.uint8)

def func_pista(frame):
    '''Use esta função para ver quanto branco tem na tela'''
    while (True):    #     # Capture frame-by-fra
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(frame_hsv, cor_menor, cor_maior)
        blur = cv2.GaussianBlur(mask_white, (5,5),0)
        print(blur.shape[0], blur.shape[1])
        counter_white = 0
        counter_black = 0
        for i in range(blur.shape[0]):
            # Para cada coluna
            for j in range(blur.shape[1]/2, blur.shape[1]):
                if blur[i][j] == np.array([255, 255, 255]):
                    counter_white += 1
                elif blur[i][j] == np.array([0,0,0]):
                    counter_black += 1
        return counter_white/counter_black
