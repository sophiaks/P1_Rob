#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import numpy as np
import math
from matplotlib import pyplot as plt
import time

cor_menor = np.array([240, 240, 240], dtype=np.uint8)
cor_maior = np.array([255, 255, 255], dtype=np.uint8)

def encontra_pista(frame):
    while (True):    #     # Capture frame-by-fra
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(frame_hsv, cor_menor, cor_maior)
        blur = cv2.GaussianBlur(mask_white, (5,5),0)
        counter_ white = 0
        counter_black = 0
        for i in range(blur.shape[0]):
            # Para cada coluna
            for j in range(blur.shape[1]):
                if blur[i][j] == np.array([255, 255, 255]):
                    counter_white += 1
                elif blur[i][j] == np.array([0,0,0]):
                    counter_black += 1
        return counter_white/counter_black
