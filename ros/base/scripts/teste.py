#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division, print_function
from cv_bridge import CvBridge, CvBridgeError
import math
import time
import cv2
import numpy as np
from numpy import linalg
import base_proj
import cormodule
import pto_fuga
import garra_demo
import rospy
import tf
import tf2_ros
import visao_module
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from geometry_msgs.msg import Pose, Twist, Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Header
from tf import TransformerROS, transformations
import garra_demo
__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


lista_quero = ['blue', 11, 'cat']
laser = []
resultados = []
lines = []
lines = None
line1 = None
line2 = None

global bordas_color
bordas = None


global ptos
global ptom
global bordas_color
global tutorial
global pegou
global mask
mask = None

cv_image = None

ptos = []
ptom = None
bordas_color = None

pegou = False
x = 0
y = 0
z = 0
id = 0


def funcscan(msg):
    global laser
    laser = msg.ranges


bridge = CvBridge()
temp_image = None

#media = []
#centro = []
atraso = 1.5E9  # 1 segundo e meio. Em nanossegundos
tutorial = garra_demo.MoveGroupPythonIntefaceTutorial()

area = 0.0  # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados.
# Descarta imagens que chegam atrasadas demais

check_delay = False
# _________________________TF_________________________#

#frame = "head_camera"
frame = "camera_link"
tfl = 0
tf_buffer = tf2_ros.Buffer()

#__________________________________________________________ FUNÇÕES EM BASE_PROJ

def recebe(msg):
    global x  # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
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
        r = transformations.quaternion_matrix(
            [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        # Criamos a matriz composta por translacoes e rotacoes
        m = np.dot(r, t)
        # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
        z_marker = [0, 0, 1, 0]
        v2 = np.dot(m, z_marker)
        v2_n = v2[0:-1]  # Descartamos a ultima posicao
        n2 = v2_n/linalg.norm(v2_n)  # Normalizamos o vetor
        x_robo = [1, 0, 0]
        # Projecao do vetor normal ao marcador no x do robo
        cosa = np.dot(n2, x_robo)
        angulo_marcador_robo = math.degrees(math.acos(cosa))

        # Terminamos
        #print("id: {} x {} y {} z {} angulo {} ".format(id, x, y, z, angulo_marcador_robo))

# _________________________________RODA_TODO_FRAME_________________


#A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global cv_image
    global resultados
    global media
    global centro
    global temp_image
    global frame
    global pto
    global linha1
    global linha2

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime  # calcula o lag
    delay = lag.nsecs
    print("delay ", "{:.3f}".format(delay/1.0E9))


    if delay > atraso and check_delay == True:
        print("Descartando por causa do delay do frame:", delay)
        return
    try:
        antes = time.clock()
        
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        #cv_image = cv2.flip(cv_image, -1)

        media, centro, maior_area = cormodule.identifica_cor(cv_image, lista_quero[0])
        centro, saida_net, resultados = visao_module.processa(temp_image)
        #pto = pto_fuga.line_intersecction(cv_image)
        for r in resultados:
            print(r)
            pass
        depois = time.clock()
        #cv2.imshow("Camera", cv_image)

    except CvBridgeError as e:
        print('ex', e)






if __name__ == "__main__":
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    

    # topico_imagem = "/kamera"
    topico_imagem = "/camera/rgb/image_raw/theora"

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # Para recebermos notificacoes de que marcadores foram vistos
    
    recebedor2 = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size=2**24)
    recebedor2 = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe)
    
    print("Usando ", topico_imagem)

    dist = rospy.Subscriber(("/scan"), LaserScan, funcscan)
    tfl = tf2_ros.TransformListener(tf_buffer)
    tolerancia = 25

    try:
        tfl = tf2_ros.TransformListener(tf_buffer)

        while not rospy.is_shutdown():     
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
            print("Girando inicialmente")
            if cv_image is not None:
            
                lines = []            
                lines = None            
                line1 = None
                line2 = None

                ptos = []
                mask  = cv2.inRange(cv_image, pto_fuga.cor_menor, pto_fuga.cor_maior)
                mask += cv2.inRange(cv_image, pto_fuga.cor_menor2, pto_fuga.cor_maior2)
                blur = cv2.GaussianBlur(mask, (5, 5), 0)
                bordas = cv2.Canny(blur, 50, 150, apertureSize=3)
                bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)
                lines = cv2.HoughLines(bordas,1,np.pi/180,100)  

                #if id != lista_quero[1] or id == None:
                if id == None:
                    print("nenhum id encontrado")
                # Segue o código do ponto de fuga
                print(lista_quero[1], id)#__________________________________________________________________CASO TENHA ALGUMA LINHA_______________________________________________________________________#

                if lines is not None:

                    print("tem linha")

                    for line in lines:
                        
                        for rho, theta in line:

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
                                print("o q q ta ocorrendo")


                            h = y0 - (m * x0)
                            p1 = (x1, y1)
                            p2 = (x2, y2)

                            print(p1, p2)

                            if m < -0.1 and m > -15:
                                cv2.line(cv_image,(x1,y1),(x2,y2),(0,255,0),1) 
                                line1 = (p1, p2)
                                print("Linha esquerda ok")

                            elif m > 0.1 and m < 15:
                                cv2.line(cv_image,(x1,y1),(x2,y2),(0,255,0),1) 
                                line2 = (p1, p2) 
                                print("Linha direita ok")

                            if line1 is not None and line2 is not None:

                                pi = pto_fuga.line_intersecion(line1, line2)
                                ptos.append(pi)

                            # elif line1 is None and line2 is not None:
                            #     vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.05))
                            #     print("esquerda (linha)")
                            # else:
                            #     vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.05))
                            #     print("direita (linha)")
                                

#_______________________________________________________________________________SE ACHAR PONTO DE FUGA_________________________________________________________________#
#Média dos pontos:   
                    if ptos is not None: # acho q n precisa, mas...
                        if len(ptos)> 0:

                            print("tem ponto!!")

                            if len(ptos) > 1:
                                ptom = np.array(ptos).mean(axis=0)

                            else:
                                ptom = ptos[0]

                            ptom = tuple(ptom)   

                            cv2.circle(cv_image, (int(ptom[0]), int(ptom[1])), 3, (255,0,0), 2) 


                            print("x do ponto: {} y do ponto {}".format(ptom[0], ptom[1]))

                            if centro[0] > cv_image.shape[0]/2 + 100:
                                vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.05))
                                                            
                                print("Ponto tá pra direita. Velocidade atual: {}".format(vel))
                                
                            elif centro[0] < cv_image.shape[0]/2 - 100:
                                vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.05))
                                
                                print("Ponto tá pra esquerda. Velocidade atual: {}".format(vel))

                            else:
                                vel = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0))
                                print("Seguindo em frente. Velocidade atual: {}".format(vel))
                                        
                        else:
                            print("Não achou o ponto de fuga")
                            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.2))
                            print("Velocidade atual: {}".format(vel))





#__________________________________________________________________ACHOU O ID DA LISTA________________________________________________________________________#

            # 1. Manter o robô na pista usando O código do pto de fuga


                elif id == lista_quero[1]:
                        #print(lista_quero[1], id)
                        #print("id encontrado")
                    if y < -0.1:
                        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.05))
                        #print("Velocidade atual: {}".format(vel))
                        #print("Girando pra direita")
                    elif y > 0.1:
                        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.05))
                        #print("Velocidade atual: {}".format(vel))
                        #print("Girando pra esquerda")
                    else:
                        #print('Indo pra frente')
                        vel = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0))
                        #print("Velocidade atual: {}".format(vel))
                        for value in laser:
                            if value < 0.15:
                                lista_dist.append(value)
                                print(lista_dist)
                        if len(lista_dist) > 40 and pegou == False:
                            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.2)
                            print("Robô perto do creeper. Parando...")
                            
                            print("\n----------------------------------------------------------")

                            print("\n============ Press `Enter` to go to init joint state ...\n")
                            raw_input()
                            tutorial.go_to_init_joint_state()


                            print("\n============ Press `Enter` to go to home joint state ...\n")
                            raw_input()
                            tutorial.go_to_home_joint_state()

                            print("\n============ Press `Enter` to open gripper  ...\n")
                            raw_input()
                            tutorial.open_gripper()

                            print("\n============ Press `Enter` to close gripper  ...\n")
                            raw_input()
                            tutorial.close_gripper()

                            print("\n============ Press `Enter` to go to init goal ...\n")
                            raw_input()
                            tutorial.go_to_zero_position_goal()


                            print("\n============ Press `Enter` to go to home goal ...\n")
                            raw_input()
                            tutorial.go_to_home_position_goal()

                            print("\n============ Press `Enter` to open gripper  ...\n")
                            raw_input()
                            tutorial.open_gripper()

                            print("\n============ Press `Enter` to close gripper  ...\n")
                            raw_input()
                            tutorial.close_gripper()

                            pegou = True


#__________________________________________________________________ACHOU UM ID MAS NÃO É O DA LISTA________________________________________________________________________#

                    
                elif pegou == True:
                    print("Achou o id errado, centralizando no ponto de fuga de novo")
                    if len(ptom) > 0:  # Se tiver um ponto de fuga
                        for res in resultados:
                            if res != lista_quero[2]:
                                print("x do ponto: {} y do ponto {}".format(ptom[0], ptom[1]))
                                if ptom[0] > cv_image.shape[0]/2 + 15:
                                    vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.2))
                                    print("Velocidade atual: {}".format(vel))
                                elif ptom[0] < cv_image.shape[0]/2 - 15:
                                    vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.2))
                                    print("Velocidade atual: {}".format(vel))
                                else:
                                    vel = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0))
                                    print("Velocidade atual: {}".format(vel))
                
                            else:
                                print("Achou o id errado, centralizando no ponto de fuga de novo")
                                if len(ptom) > 0:  # Se tiver um ponto de fuga
                                    print("x do ponto: {} y do ponto {}".format(
                                        ptom[0], ptom[1]))
                                    if ptom[0] > cv_image.shape[0]/2 + 15:
                                        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.2))
                                        print("Velocidade atual: {}".format(vel))
                                    elif ptom[0] < cv_image.shape[0]/2 - 15:
                                        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.2))
                                        print("Velocidade atual: {}".format(vel))
                                    else:
                                        vel = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0))
                                        print("Velocidade atual: {}".format(vel))
                    else:
                        print("ué cadê o ponto de fuga")                   


            
            


            # Alinhado com o while not shutdown
            velocidade_saida.publish(vel)
            rospy.sleep(0.2)

            # if cv_image is not None:                
            #     cv2.imshow("/camera/rgb/image_raw/theora", cv_image)
            #     cv2.waitKey(1)

            if bordas_color is not None:
                numpy_vertical_concat = np.concatenate((bordas_color, cv_image), axis=1)
                cv2.imshow("Video", numpy_vertical_concat)
                cv2.waitKey(1)
                

            if cv2.waitKey(1) & 0xFF == ord('q'):
                 break

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")

        #oi sophoa












