#!/usr/bin/env python
import rospy
from traversal.msg import WheelRpm
from detection.srv import *
import sensor_msgs.msg as sensor_msgs
import cv2
import numpy as np

class ArrowNode():

	def __init__(self):
		rospy.init_node('arrow_det',anonymous=False)
		rospy.Subscriber ('imu', sensor_msgs.Imu, self.imu_callback)
		self.bearing = 0
		
		self.lefttemplate=cv2.imread("/home/neel/caesar2020/src/detection/left2.jpg",0)
		self.lefttemplate=cv2.GaussianBlur(self.lefttemplate,(13,13),0)
		self.righttemplate=cv2.imread("/home/neel/caesar2020/src/detection/right2.jpg",0)
		self.righttemplate=cv2.GaussianBlur(self.righttemplate,(13,13),0)

		self.w, self.h = self.righttemplate.shape[::-1] 
		self.video = cv2.VideoCapture(0)
		
		
		self.pub=rospy.Publisher('drive_inp',WheelRpm,queue_size=10)
		self.rate=rospy.Rate(10)
		self.rate_90=rospy.Rate(0.15)  		#FIND TIME FOR 90 DEGREE TURN
		self.rate_str=rospy.Rate(0.28)
		self.rpm = WheelRpm()
		self.top_left_l=0
		self.top_left_r=0
		self.x_n=0
		self.x_below=0
		self.x_above=0
		self.false_count=0
		self.exit=0
		self.lr = rospy.Service('arrow', arrow ,self.detect)

	def detect(self,msg):
		while True:
			
			foundright = 0
			foundleft=0
			ret,img=self.video.read()
			img_gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
			for scale in np.linspace(0.05, 1.0, 10)[::-1]:
				width = int(img.shape[1] * scale) 
				height = int(img.shape[0] * scale) 
				dim = (width, height) 
				resized = cv2.resize(img_gray,dim)
				resized=cv2.GaussianBlur(resized,(13,13),0)
		    
				edges = resized
		    
		    
				result = cv2.matchTemplate(edges, self.righttemplate, cv2.TM_CCOEFF)
				(_, maxValright, _, maxLocright) = cv2.minMaxLoc(result) 
		    
				if  maxValright > foundright: 
					foundright = maxValright
					s_r=scale
					top_left_r=maxLocright[0]
			
		     
				result = cv2.matchTemplate(edges, self.lefttemplate, cv2.TM_CCOEFF)
				(_, maxValleft, _, maxLocleft) = cv2.minMaxLoc(result) 
		    
				if  maxValleft > foundleft: 
					foundleft = maxValleft
					s_l=scale
					top_left_l=maxLocleft[0]
					
			print foundright
			print top_left_r
			print s_r
			   
		   
			if foundright>7000000 or foundleft>7000000:
				self.exit=0
		    
				if foundright > foundleft:
					x_r=int((top_left_r+(self.w/2))/(s_r))
					print "Right"
					print x_r
					
					if 300<x_r<340:
						self.rpm.vel=15
						self.rpm.omega=0
						self.rpm.hb=False
						self.pub.publish(self.rpm)
						self.x_n=self.x_n+1
					elif x_r<=300:
						self.rpm.vel=0
						self.rpm.omega=12
						self.rpm.hb=False
						self.pub.publish(self.rpm)
						self.x_below=self.x_below+1
					else:
						self.rpm.vel=0
						self.rpm.omega=-12
						self.rpm.hb=False
						self.pub.publish(self.rpm)
						self.x_above=self.x_above+1
						
					if self.h/s_r>=80 and self.rpm.vel==15:
						self.rpm.vel=0
						self.rpm.omega=12
						self.rpm.hb=False
						self.pub.publish(self.rpm)
						print "turning right"
						self.wait_for_deviation (80)
						self.x_n=0
						self.x_above=0
						self.x_below=0
						
						
						print "going straight"
						self.rpm.vel=15
						self.rpm.omega=0
						self.rpm.hb=False
						self.pub.publish(self.rpm)
						self.rate_str.sleep()
						
					
			    
				else:
					x_l=int((top_left_l+(self.w/2))/(s_l))
					print "Left"
					#print x_l
				
					
					if 300<x_l<340:
						self.rpm.vel=15
						self.rpm.omega=0
						self.rpm.hb=False
						self.pub.publish(self.rpm)
						self.x_n=self.x_n+1
					elif x_l<=300:
						self.rpm.vel=0
						self.rpm.omega=12
						self.rpm.hb=False
						self.pub.publish(self.rpm)
						self.x_below=self.x_below+1
					else:
						self.rpm.vel=0
						self.rpm.omega=-12
						self.rpm.hb=False
						self.pub.publish(self.rpm)
						self.x_above=self.x_above+1
						
					if self.h/s_l>=80 and self.rpm.vel==15:
						self.rpm.vel=0
						self.rpm.omega=-12
						self.rpm.hb=False
						print "turning left"
						self.wait_for_deviation (80)
						self.x_n=0
						self.x_above=0
						self.x_below=0
						
						print "going straight"
						self.rpm.vel=15
						self.rpm.omega=0
						self.rpm.hb=False
						self.pub.publish(self.rpm)
						self.rate_str.sleep()
						
					
			    
			else:
				print("no arrow")
						
				self.exit=self.exit+1
				
				if self.x_n > self.x_below and self.x_n > self.x_above and self.false_count<2:
					self.rpm.vel=15
					self.rpm.omega=0
					self.rpm.hb=False
					self.pub.publish(self.rpm)
					self.false_count=self.false_count+1
				
				elif self.x_above > self.x_n and self.x_above> self.x_below and self.false_count<2:
					self.rpm.vel=0
					self.rpm.omega=-12
					self.rpm.hb=False
					self.pub.publish(self.rpm)
					self.false_count=self.false_count+1
					
				elif self.x_below > self.x_n and self.x_below> self.x_above and self.false_count<2:
					self.rpm.vel=0
					self.rpm.omega=12
					self.rpm.hb=False
					self.pub.publish(self.rpm)
					self.false_count=self.false_count+1
				
				else :
					self.rpm.vel=0
					self.rpm.omega=12
					self.rpm.hb=False
					self.pub.publish(self.rpm)
					
				if self.false_count>=1:
					self.false_count=0
					self.x_n=0
					self.x_above=0
					self.x_below=0	
					
				if self.exit>120:
					break
				
  
			cv2.imshow('arrow',img)

		    # Press Q on keyboard to stop recording
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
				
		return "call service /reached"
		
	def wait_for_deviation (self, angle):
		curr = self.bearing
		while True:
			dev = (curr - self.bearing) % 360
			if dev > 180:
				dev = 360 - dev
			if dev > angle:
				break
		
	def spin(self):
		while True:
			pass
			
	def imu_callback (self, msg):
		self.bearing = msg.yaw
			
				
if __name__=='__main__':
	arr=ArrowNode()
	arr.spin()
	
        
# practical parameters: rate_90, rate, false_count threshold, 3000000,5000000, self.exit
