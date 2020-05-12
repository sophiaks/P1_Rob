#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import tf
import cv2
import time
import tf2_ros
import math
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from std_msgs.msg import Header
from tf import transformations
from tf import TransformerROS

from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header

import visao_module
import pto_fuga
import encontra_pista


bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0

frame = "camera_link"
#frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()


#________________________________________________RECEBE_________________________________________#


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



# A função a seguir é chamada sempre que chega um novo frame

#____________________________________________________RODA_TODO_FRAME____________________________________________________#


linhas1 = None
linhas2 = None
pto = []
ratio = None
   
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    # global resultados
    # global ratio
    # global pto
    # global linhas1
    # global linhas2

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        centro, saida_net, resultados =  visao_module.processa(temp_image)        
        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass

        depois = time.clock()
        # Desnecessário - Hough e MobileNet já abrem janelas
        cv_image = saida_net.copy()
    except CvBridgeError as e:
        print('ex', e)


#_______________________________________________________MAIN______________________________________________________#


    
if __name__ == "__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/rgb/image_raw/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size=2**24)
    #recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) 

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    tolerancia = 25

    try:
        # Inicializando - por default gira no sentido anti-horário
        vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))
        while not rospy.is_shutdown():
            # for r in resultados:
            #     print(r)    
            # 

            #ratio = None

            lines = []
            lines = None     

            line1 = None
            line2 = None
            ptos = []  


                
            if cv_image is not None:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gray,(5,5),0)

                bordas = cv2.Canny(blur,50,150,apertureSize = 3)
                bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)  


                
                mask = cv2.inRange(bordas_color, pto_fuga.cor_menor, pto_fuga.cor_maior) 
                
                lines = cv2.HoughLines(mask,1,np.pi/180,200)
                
                #framed = None 
                

                if lines is not None:

                    for line in lines:            
                        
                        for rho,theta in line:
                            
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
                            p1 = (x1,y1)
                            p2 = (x2,y2)
                                
                            
                            if m < -0.4 and m > -1.4:
                                cv2.line(cv_image,(x1,y1),(x2,y2),(0,255,0),1) 
                                line1 = (p1, p2)
                                
                            elif m > 0.3 and m < 2.1:
                                cv2.line(cv_image,(x1,y1),(x2,y2),(0,255,0),1) 
                                line2 = (p1, p2)                   
                                
                                
                            if line1 is not None and line2 is not None:
                                
                                pi = pto_fuga.line_intersecion(line1, line2)
                                ptos.append(pi)


                    if len(ptos)> 0:
                        
                        ptos = np.array(ptos)
                        
                        print(ptos)
                                                
                        
                        if len(ptos) > 1:
                            ptom = ptos.mean(axis = 0)  
                            
                        else:
                            ptom = ptos[0]
                        
                        
                        ptom[0] = int(ptom[0])
                        ptom[1] = int(ptom[1])                   
                        
                            
                        ptom = tuple(ptom)

                        if pto[0] > cv_image.shape[0]/2 + 10:
                            vel = Twist(Vector3(0,0,0), Vector3(0,0, 0.5))

                        elif pto[0] < cv_image.shape[0]/2 - 10:
                            vel = Twist(Vector3(0,0,0), Vector3(0,0, -0.5))
                        
                        else:
                            vel = Twist(Vector3(0.5,0,0), Vector3(0,0, 0))

                        w, h = cv_image.shape()                    
                            
                        cv2.circle(cv_image, (int(ptom[0]), int(ptom[1])), 3, (255,0,0), 2)    

                        cv2.line(cv_image, (w/2 - 10, 0), (w/2 - 10, h), (255, 0, 0), 2)
                        cv2.line(cv_image, (w/2 + 10, 0), (w/2 + 10, h), (255, 0, 0), 2)
                        cv2.circle(cv_image, pto[0],2, (0,0,255), 3)

                    


            #if cv_image is not None:

                
                # Note que o imshow precisa ficar *ou* no codigo de tratamento de eventos *ou* no thread principal, não em ambos
                #cv2.imshow("cv_image no loop principal", cv_image)
                #cv2.waitKey(1)

                # if len(pto) > 0:
                #     if pto[0] > cv_image.shape[0]/2 + 10:
                #         vel = Twist(Vector3(0,0,0), Vector3(0,0, 0.5))
                #     elif pto[0] < cv_image.shape[0]/2 - 10:
                #         vel = Twist(Vector3(0,0,0), Vector3(0,0, -0.5))
                #     else:
                #         vel = Twist(Vector3(0.5,0,0), Vector3(0,0, 0))

                #     w, h = cv_image.shape()


            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


            velocidade_saida.publish(vel)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


