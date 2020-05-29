#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division, print_function
from geometry_msgs.msg import Pose, Twist, Vector3, Vector3Stamped
from sensor_msgs.msg import CompressedImage, Image, LaserScan

global lista_dist
lista_dist = []

def funcscan(msg):
    global laser
    laser = msg.ranges()

def centraliza(pto, threshold, frame):
    '''
    Centraliza o robô em um PONTO.
    '''
    if pto > frame.shape[0]/2 + threshold:
        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.2))
        print("Velocidade atual: {}".format(vel))
    elif pto < frame.shape[0]/2 - threshold:
        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.2))
        print("Velocidade atual: {}".format(vel))
    else:
        vel = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0))
        print("Velocidade atual: {}".format(vel))

def centraliza_id(pto, threshold):
    '''
    Centraliza o robô em um ID.
    '''
    if pto < -threshold:
        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.05))
        print("Velocidade atual: {}".format(vel))
        print("Girando pra direita")
    elif pto > threshold:
        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.05))
        print("Velocidade atual: {}".format(vel))
        print("Girando pra esquerda")
    else:
        print('Indo pra frente')
        vel = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0))
        print("Velocidade atual: {}".format(vel))

        for value in laser:
            if value < 0.18:
                lista_dist.append(value)

def centraliza_obj(lista, frame, resultados):
    '''
    Centraliza o robô em qualquer resultado.
    '''
    if lista[2] == "cat" and resultados[0] == "cat":
        (centrox, centroy) = (resultados[3] - resultados[2])/2
        if centroy > frame.shape[0]/2 + 15:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.2))
            print("Velocidade atual: {}".format(vel))
        elif centroy < frame.shape[0]/2 - 15:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.2))
            print("Velocidade atual: {}".format(vel))
        else:
            vel = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0))
            print("Velocidade atual: {}".format(vel))
    elif lista_quero[2] == "bird":
        (centrox, centroy) = (resultados[3] - resultados[2])/2
        if centroy > frame.shape[0]/2 + 15:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.2))
            print("Velocidade atual: {}".format(vel))
        elif centroy < frame.shape[0]/2 - 15:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.2))
            print("Velocidade atual: {}".format(vel))
        else:
            vel = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0))
            print("Velocidade atual: {}".format(vel))
    elif lista_quero[2] == "bycicle":
        (centrox, centroy) = (resultados[3] - resultados[2])/2
        if centroy > frame.shape[0]/2 + 15:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.2))
            print("Velocidade atual: {}".format(vel))
        elif centroy < frame.shape[0]/2 - 15:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.2))
            print("Velocidade atual: {}".format(vel))
        else:
            vel = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0))
            print("Velocidade atual: {}".format(vel))
    elif lista_quero[2] == "dog":
        (centrox, centroy) = (resultados[3] - resultados[2])/2
        if centroy > frame.shape[0]/2 + 15:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.2))
            print("Velocidade atual: {}".format(vel))
        elif centroy < frame.shape[0]/2 - 15:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.2))
            print("Velocidade atual: {}".format(vel))
        else:
            vel = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0))
            print("Velocidade atual: {}".format(vel))
