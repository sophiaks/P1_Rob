#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cormodule
from __future__ import print_function, division
import rospy
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from std_msgs.msg import Header

import visao_module

lista_quero = []
laser = []
resultados = []

x = 0
y = 0
z = 0 
id = 0


def funcscan(msg):
	global laser
	laser = msg.ranges
	print(laser)

bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 
#_________________________TF_________________________
frame = "camera_link"
tfl = 0
tf_buffer = tf2_ros.Buffer()


def recebe(msg):
	global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
	global y
	global z
	global id
	for marker in msg.markers:
		id = marker.id
		marcador = "ar_marker_" + str(id)

		print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
		header = Header(frame_id=marcador)
		# Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
		# Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a 
		# Nao ser que queira levar angulos em conta
		trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
		
		# Separa as translacoes das rotacoes
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		# ATENCAO: tudo o que vem a seguir e'  so para calcular um angulo
		# Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
		# no eixo X do robo (que e'  a direcao para a frente)
		t = transformations.translation_matrix([x, y, z])
		# Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
		r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
		m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
		z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
		v2 = numpy.dot(m, z_marker)
		v2_n = v2[0:-1] # Descartamos a ultima posicao
		n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
		x_robo = [1,0,0]
		cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
		angulo_marcador_robo = math.degrees(math.acos(cosa))

		# Terminamos
		print("id: {} x {} y {} z {} angulo {} ".format(id, x,y,z, angulo_marcador_robo))

#_________________________________RODA_TODO_FRAME_________________


# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global resultados
	global media
	global centro

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		cv_image = cv2.flip(cv_image, -1)
        
		media, centro, maior_area =  cormodule.identifica_cor(cv_image)
        centro, saida_net, resultados =  visao_module.processa(temp_image)
        for r in resultados:
            print(r)
            pass
        depois = time.clock()
		cv2.imshow("Camera", cv_image)


	
	except CvBridgeError as e:
		print('ex', e)
	
if __name__=="__main__":
	rospy.init_node("cor")

	# topico_imagem = "/kamera"
	topico_imagem = "/camera/rgb/image_raw/compressed"
	

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	
    print("Usando ", topico_imagem)
	
    dist = rospy.Subscriber(("/scan"), LaserScan, funcscan)
    tfl = tf2_ros.TransformListener(tf_buffer)
    tolerancia = 25

	try:
        tfl = tf2_ros.TransformListener(tf_buffer)
        vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))

		while not rospy.is_shutdown():
            for r in resultados:
                print("opa")
            velocidade_saida.publish(vel)
            #1. Manter o robô na pista usando O código do pto de fuga
            if id is None:
                # Segue o código do ponto de fuga
                if len(pto) > 0: #Se tiver um ponto de fuga
                    if pto[0] > cv_image.shape[0]/2 + 10:
                        vel = Twist(Vector3(0,0,0), Vector3(0,0, 0.5))
                    elif pto[0] < cv_image.shape[0]/2 - 10:
                        vel = Twist(Vector3(0,0,0), Vector3(0,0, -0.5))
                    else:
                        vel = Twist(Vector3(0.5,0,0), Vector3(0,0, 0))

					#Fazendo linhas de limitis
                    w, h = cv_image.shape()
                    cv2.line(cv_image, (w/2 - 10, 0), (w/2 - 10, h), (255, 0, 0), 2)
                    cv2.line(cv_image, (w/2 + 10, 0), (w/2 + 10, h), (255, 0, 0), 2)
                    cv2.circle(cv_image, pto[0],2, (0,0,255), 3)

            # for value in laser:
            # 	print("nice")
            # 	if value < 1.75:
            # 		vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            # 		velocidade_saida.publish(vel)

            # else:
            #     if len(media) != 0 and len(centro) != 0:
            #         print("Média dos verdes: {0}, {1}".format(media[0], media[1]))
            #         print("Centro dos verdes: {0}, {1}".format(centro[0], centro[1]))
            #         if (media[0] > centro[0]):
            #             vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.3))
            #             if (media[0] - centro[0]) < 10:
            #                 vel = Twist(Vector3(0.3,0,0), Vector3(0,0,0))
                        

            #         if (media[0] < centro[0]):
            #             vel = Twist(Vector3(0,0,0), Vector3(0,0,0.5))
            #             if (centro[0] - media[0]) < 10:
            #                 vel = Twist(Vector3(0.3,0,0), Vector3(0,0,0))

            else: 
                for item in lista_quero:
                    if id == item:
                        print(olá)

			#Alinhado com o while not shutdown 
            velocidade_saida.publish(vel)
			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")