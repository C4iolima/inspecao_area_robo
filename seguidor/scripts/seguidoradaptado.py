#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()

def __init__(self):
  self.bridge = cv_bridge.CvBridge()
  cv2.namedWindow("window", 1)
  #a linha a seguir foi podificado o subscriber
  self.image_sub = rospy.Subscriber('/p3dx/front_camera/image_raw', cv_image, self.image_callback)
  # linha a seguir comentada e substiuida por /p3dx/cmd_vel
  # self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
  self.cmd_vel_pub = rospy.Publisher('/p3dx/cmd_vel',Twist, queue_size=1)
  self.twist = Twist()

def image_callback(ros_image):
  print 'Imagem Capturada'
  global bridge
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)
  #from now on, you can work exactly like with opencv
  #(rows,cols,channels) = cv_image.shape
  #if cols > 200 and rows > 200 :
  #    cv2.circle(cv_image, (100,100),90, 255)
  #font = cv2.FONT_HERSHEY_SIMPLEX
  #cv2.putText(cv_image,'P3DX - Seguidor de Linha!',(10,350), font, 1,(30,255,255),2,cv2.LINE_AA)
  cv2.imshow("Camera Frontal do P3DX", cv_image)
  cv2.waitKey(3)

#mostra imagem em hsv
  print 'Imagem Convertida HSV'
  hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  cv2.imshow("Imagem HSV",hsv)
#define limites superiores e inferiores de amarelo 
  yellowLower =(30, 150, 100)
  yellowUpper = (50, 255, 255)

  mask = cv2.inRange(hsv, yellowLower, yellowUpper)
  print 'Mascara aplicada'
  cv2.imshow("mask image", mask)
#aqui ele espera apertar uma tecla para atualizar para aplicacao no controle do robo remover o waitkey
 

######## a partir daqui copiado do livro
  h, w, d = cv_image.shape
  print cv_image.shape
  search_top = 3*h/4
  # print search_top
  search_bot = search_top + 20
  mask[0:search_top, 0:w] = 0
  mask[search_bot:h, 0:w] = 0
  M = cv2.moments(mask)
  if M['m00'] > 0:
   cx = int(M['m10']/M['m00'])
   cy = int(M['m01']/M['m00'])
   cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)
   #CONTROLE PROPORCIONAL CALCULANDO O ERRO A PARTIR DO MEIO DA LINHA
   err = cx - w/2
   self.twist.linear.x = 0.2
   # self.twist.angular.z = -float(err) / 100
   # self.cmd_vel_pub.publish(self.twist)
   #controle proporcinal terminou na linha de cima
   cv2.imshow("window", cv_image)
   cv2.waitKey(3)

#rospy.init_node('seguidoradaptado')
 # seguidoradaptado = seguidoradaptado()
  #rospy.spin()

#copia livro ate aqui

  cv2.waitKey(0)
  #cv2.destroyAllWindows()

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  image_sub = rospy.Subscriber("/p3dx/front_camera/image_raw",Image, image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


