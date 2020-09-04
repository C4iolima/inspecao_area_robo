#!/usr/bin/env python

#comando rosservice call /gazebo/reset_simulation
#python aulagazebo.py dentro de .../scripts/
#listar os topicos => rostopic list

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np



Laser_msg = None
anglemin = None
anglemax = None
#vetor que mostra os feixes do laser scan
ranges = None

#nao esta sendo usado 
angles = None
xr=None
yr=None
thr=None

#para o codigo parar por alguns instantes
def sleep(t):
    try:
        rospy.sleep(t)
    except:
        pass
 
#definicao do subscriber. Pega os dados do laser scan e coloca no vetor ranges       
def callback_laser(msg): 
    global Laser_msg
    global ranges
    Laser_msg=msg
    ranges = Laser_msg.ranges
    
#Pega os dados de odometria do robo (x,y,teta)
def callback_odom(msg): 
    global xr
    global yr
    global thr
    xr=msg.pose.pose.position.x
    yr=msg.pose.pose.position.y
    thr = np.arctan2(2*msg.pose.pose.orientation.w*msg.pose.pose.orientation.z,1-2*msg.pose.pose.orientation.z*msg.pose.pose.orientation.z); 

#Envia os sinais de controle do robo    
def talker():
         
    
    ###### SETUPPP #########
#publica comando de velocidade
    pub_vel = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size = 1)
#inicializa o no
    rospy.init_node('aulagazebo', anonymous=False)
#chama as funcoes callback_laser e callback_odom e associa com o robo
    rospy.Subscriber('/p3dx/front_laser/scan', LaserScan, callback_laser) 
    rospy.Subscriber('/p3dx/odom',Odometry,callback_odom)  
    rate = rospy.Rate(10) # 10hz
    
    
    rospy.loginfo("1. Starting! ")
#inicializa em zero as velocidades
    # Velocity Message    
    twist = Twist()
    twist.linear.x = 0 
    twist.linear.y = 0 
    twist.linear.z = 0
    twist.angular.x = 0 
    twist.angular.y = 0 
    twist.angular.z = 0
    #rospy.sleep(1) 
    rate.sleep()

    xr = 0
    
   # # Rangefinder Angles
   # min_angle=Laser_msg.angle_min
   # #max_angle=Laser_msg.angle_max
   # increment=Laser_msg.angle_increment
   # n_angles=int(round((max_angle-min_angle)/increment)+1)
   # angles=min_angle+np.array(range(n_angles))*(max_angle-min_angle)/(n_angles-1)
    

   # Goal
   # Desired point to start
    xg = 0.0
    yg = 0.0
    thg = 0.0

    #Desired variables
    w = 0
    v = 0
    mDist = 0
    mAng = 0
    thrshold = 0

    direction = 1
    pdirection = 0
    numdirection = 0

    rospy.loginfo("2. Looping ")
    rospy.loginfo(" P[xr,yr,thr] C[lin.x,ang.z] A[mDist,mAng] Start End")
    
    ######## LOOOP  ########
    while not rospy.is_shutdown(): 

        #Search for the point where laser scan is inf
        
        maxRange = 0  #what is the max angle (outside should be at max) (feixe maximo tera a maior distancia)
        angStart = 0  #where it starts (angulo de inicio da porta)
        angEnd   = 0  #where it ends (angulo final da porta)
        
        i = 0
        while i < len(ranges):
            if ranges[i] > maxRange: #se a distancia encontrada for maior que a distancia anterior encontrada
                maxRange = ranges[i] #maxRange atualiza
                angStart = i #angulo inicial eh atualizado
                #rospy.loginfo("Max [%f] [%d - %d]  ",maxRange,angStart,angEnd)
            else:
                if ranges[i] == maxRange: #se nao houver distancia (ou seja maxRange eh igual a distancia encontrada)
                    angEnd = i #atualiza angulo final
                    #rospy.loginfo("Max [%f] [%d - %d]  ",maxRange,angStart,angEnd)
            i += 1  
        else:
        


            #Mean dist should be a good reference
            if ( (angStart>1) and (angEnd<360) ):
#distancia media entre as duas arestas da porta. Diminui um grau de angStart e aumenta um grau de angEnd (pq esses angulos sao infinitos)
                mDist = ranges[angStart-1]/2 + ranges[angEnd+1]/2
                mAng = (angEnd - angStart)*0.0174533

	#3/4 de pi
	thr2 = thr + 2.35
	# #controle proporcional no eixo z  (fator 0.03)
	# twist.angular.z = (mAng - thr2)*0.0171
	# twist.linear.x = 0.09





	if ((mDist>0.80)):
 #                #at this point the door should be between angStart and angEnd. However those are related to the sensor viewpoint
                if angEnd>angStart:
                    mAng = (angEnd - angStart)*0.0174533  # the 10 degreee added intends to find point close to the door                   
		   

	# 	   # if the robot is lookin to the right
		if ((thr > 0.1) ):
		# and target is to the right
			if ((angStart)< (2.9) ):
				mAng = 0.0
                   	else:
	                        mAng = 0.5
		else:
			if (thr < -0.1): #if is lookint to the left
			# and target is to the right
				if ((angStart)< (2.9) ):
	                        	mAng = -0.5
	                   	else:
		                        mAng = 0.0

			else: # has to be looking foward
	                        # and target is to the right
				if ((angStart)< (2.9) ):
	                        	mAng = 0.1
	                   	else:
					if ((angStart)> (3.0) ):
		                        	mAng = -0.1
					else:
		                        	mAng = 0.0



		if ((thr > 0.6) ):
			mAng = 0

		if ((thr < -0.6) ):
			mAng = 0
		else:
			if angEnd>angStart:
				mAng = ((angEnd - angStart) - 50)*0.0174533 # the 10 degreee added intends to find point close to the door
			else:
                    		mAng = 90*0.0174533
				thg =  mAng
				xg  = xg + mDist*np.sin(mAng) * 0.1
				yg  = yg + mDist*np.cos(mAng) * 0.1
			if ((mDist<0.8)): 
                		thg = thr
		        if (mDist>1.1):
		            twist.linear.x = 00.05
		            twist.linear.x  = 0    
		            twist.angular.z = (thg) * 0.5
		        else:
        		    if (mDist>0.7):
        		        twist.linear.x  = 00.025
        		        # twist.angular.z = (thg - thr) * 0.5 
                        twist.angular.z = (mAng - thr2)*0.0171
                    else:
                        twist.linear.x  = 00.01
                        twist.angular.z = (mAng - thr2)*0.0171
                                            # twist.angular.z = 0
       # TO help understand controll
        rospy.loginfo(" P[%.2f,%.2f,%.3f] C[%.2f,%.2f] A[%.2f,%.2f] %.2f %.2f",xr,yr,thr,twist.linear.x,twist.angular.z,mDist,mAng,angStart*0.0174533,angEnd*0.0174533)

        

        pub_vel.publish(twist)
        rate.sleep()

            

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
