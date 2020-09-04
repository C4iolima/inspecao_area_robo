import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

#PARTE SEGUIDOR COM CAMERA

class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		self.image_sub = rospy.Subscriber('/p3dx/front_camera/image_raw',Image, self.image_callback)
		self.cmd_vel_pub = rospy.Publisher('/p3dx/cmd_vel',Twist, queue_size=1)		
		# self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)
		self.twist = Twist()
def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_yellow = numpy.array([ 10, 10, 10])
		upper_yellow = numpy.array([255, 255, 250])
		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
		h, w, d = image.shape
		search_top = 3*h/4
		search_bot = 3*h/4 + 20
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0
		M = cv2.moments(mask)
		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			# print M['m00']
			# print M['m10']
			# print M['m01']
			#Coloca um circulo solido onde calcula que e o centro da linha
			cv2.circle(image, (cx, cy), 5, (0,0,255), -1)
			err = cx - w/2
			print 'CX'
			print cx
			print 'Centro da tela em'
			print w/2
			print ' '
			#Comeca a se deslocar
			self.twist.linear.x = 0.10
			#Se o erro ficar positivo o robo gira no sentido anti-horario
			if err > 10:
				self.twist.angular.z = -float(err) / 1000
				self.cmd_vel_pub.publish(self.twist)
			#Se o erro ficar negativo o robo gira no sentido horario
			if err < 10:
				self.twist.angular.z = float(err) / 1000
				self.cmd_vel_pub.publish(self.twist)
		cv2.imshow("window", image)
		cv2.waitKey(3)
rospy.init_node('follower')
follower = Follower()
rospy.spin()

#################################################################################################################################################

#PARTE LASER SCAN

Laser_msg = None
anglemin = None
anglemax = None
ranges = None
angles = None
xr=None
yr=None
thr=None

def sleep(t):
    try:
        rospy.sleep(t)
    except:
        pass
 
       
def callback_laser(msg): 
    global Laser_msg
    global ranges
    Laser_msg=msg
    ranges = Laser_msg.ranges
    

def callback_odom(msg): 
    global xr
    global yr
    global thr
    xr=msg.pose.pose.position.x
    yr=msg.pose.pose.position.y
    thr = np.arctan2(2*msg.pose.pose.orientation.w*msg.pose.pose.orientation.z,1-2*msg.pose.pose.orientation.z*msg.pose.pose.orientation.z); 

    
def talker():
         
    
    ###### SETUPPP #########
    pub_vel = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size = 1)
    rospy.init_node('aulagazebo', anonymous=False)
    rospy.Subscriber('/p3dx/front_laser/scan', LaserScan, callback_laser) 
    rospy.Subscriber('/p3dx/odom',Odometry,callback_odom)  
    rate = rospy.Rate(10) # 10hz
    
    
    rospy.loginfo("1. Starting! ")

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
        
        maxRange = 0  #what is the max angle (outside should be at max)
        angStart = 0  #where it starts
        angEnd   = 0  #where it ends
        
        i = 0
        while i < len(ranges):
            if ranges[i] > maxRange:
                maxRange = ranges[i]
                angStart = i
                #rospy.loginfo("Max [%f] [%d - %d]  ",maxRange,angStart,angEnd)
            else:
                if ranges[i] == maxRange:
                    angEnd = i
                    #rospy.loginfo("Max [%f] [%d - %d]  ",maxRange,angStart,angEnd)
            i += 1  
        else:
        


            #Mean dist should be a good reference
            if ( (angStart>1) and (angEnd<360) ):
                mDist = ranges[angStart-1]/2 + ranges[angEnd+1]/2 

            if ((mDist>0.80)):
                #at this point the door should be between angStart and angEnd. However those are related to the sensor viewpoint
                if angEnd>angStart:
                   #mAng = (angEnd - angStart)*0.0174533  # the 10 degreee added intends to find point close to the door                   
		   

		   # if the robot is lookin to the right
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
            #twist.linear.x  = 0    
            twist.angular.z = (thg) * 0.5
        else:
            if (mDist>0.7):
                twist.linear.x  = 00.025
                twist.angular.z = (thg - thr) * 0.5 
            else:
                twist.linear.x  = 00.01
                twist.angular.z = 0
        
        # TO help understand controll
        rospy.loginfo(" P[%.2f,%.2f,%.3f] C[%.2f,%.2f] A[%.2f,%.2f] %.2f %.2f",xr,yr,thr,twist.linear.x,twist.angular.z,mDist,mAng,angStart*0.0174533,angEnd*0.0174533)

        

        pub_vel.publish(twist)
        rate.sleep()

            

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
