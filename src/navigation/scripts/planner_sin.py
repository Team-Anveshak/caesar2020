#!/usr/bin/env python
#add service to reload params
import rospy
from navigation.srv import *
from navigation.msg import *
from sensors.msg import *
from traversal.msg import *
from traversal.srv import *
from termcolor import colored
import numpy as np
import sys, signal,thread

import matplotlib.pyplot as plt

class Planner():
	
	def __init__ (self):
		rospy.init_node("Planner")
		self.load_vars() #variables
		self.load_params() #param variables

		#subscribers
		try:
			rospy.Subscriber("imu",Imu, self.imuCallback)
			rospy.Subscriber("goal",Goal,self.goalCallback)
			#rospy.Subscriber("scan",LaserScan,self.rplCallback)
		except Exception,e:
			print e

		#publishers
		self.pub_drive=rospy.Publisher("drive_inp",WheelRpm,queue_size=10)
		self.pub_planner_state=rospy.Publisher("planner_state",Planner_state,queue_size=2)

		#service server
		self.state_ser=rospy.Service('Planner_state_ctrl',plan_state,self.state_ctrl) #state service

	def spin(self):
		rate = rospy.Rate(25)		
		while not rospy.is_shutdown():
			self.main() #main func
			rate.sleep()

	def main(self):
		if(self.state=="run"):
			#if (self.distance_to_dest > self.dist_tolerance): 
				#self.obs_scanner_active = False					
				#self.pub_planner_state.publish(0)

				#if self.iter == 0 :
				#	print 1
				#	result = self.rotator()
				#	print result

				#	ax=plt.figure()	
				#	plt.ylim(-180 , 180)
				#	plt.xlim(0,250)
				#	self.iter = self.iter+1

				#if abs(self.bearing_dest-self.bearing_curr)>3*self.bearing_tolerance:
				#	result = self.rotator()
				#	print result
				#else:				
			'''plt.plot(self.tsys,self.bearing_dest,'g.') 
					plt.plot(self.tsys,self.bearing_curr,'r.')
					plt.pause(.0001)'''

			error=self.bearing_dest#-self.bearing_curr
			'''print error
					if error>180 :
						error = error - 360
					elif error<-180 :
						error = error + 360'''
				
			self.omega = self.output(error)
			self.vel = self.forward_vel_cal(1.5)
			self.drive_pub()

			#else :
			#	self.pub_planner_state.publish(1)
			#	self.obs_scanner_active = False
			#	rospy.loginfo("Jai RTN")

		elif(self.state=="pause"):
			pass
			
		elif(self.state=="stop"):
			self.vel = 0
			self.omega = 0 
			self.drive_pub()
			pass

	def state_ctrl(self,srv_msg):
		if (srv_msg.pause==1 and srv_msg.contin==0 ) :
			self.state = "pause"
		elif (srv_msg.contin==1 and srv_msg.pause==0):
			self.state = "run"
		elif (srv_msg.rst==1):
			self.state = "stop"
		else:
			rospy.loginfo("Error in changing planner state")
		#print(srv_msg.contin)
		return plan_stateResponse(self.state)

	def rotator(self):		
		while (abs( ((-1.34114064173*180)/3.14) - self.bearing_curr) > self.bearing_tolerance):
			#print 2
			remainAngle = ((-1.34114064173*180)/3.14) - self.bearing_curr
						
			if remainAngle>180 :
				remainAngle = remainAngle - 360
			elif remainAngle<-180 :
				remainAngle = remainAngle + 360
			
			omega_tmp = 17 + abs(remainAngle)/8
	
			if remainAngle<0:
				self.omega = int(omega_tmp)
			else:
				self.omega = - int(omega_tmp)
			self.vel = 0
			self.drive_pub()

		rate = rospy.Rate(2)
		self.vel = 0
		self.omega = 0
		self.drive_pub()
		rate.sleep()
		return "Rotate_finished - error=%f"%(self.bearing_dest-self.bearing_curr)

	def output(self,error):
		tim = rospy.get_time()
		dt = tim-self.time_prev

		if(self.time_prev==0):
			dt=0

		self.tsys+=dt

		output_p = error*self.kp
		if(dt!=0):
			output_d=((error-self.error_prev)/dt)*self.kd
		else:
			output_d=0
		self.error_int=self.error_int+error*dt
		output_i=self.error_int*self.ki
		output = -(output_p+output_d+output_i)

		self.time_prev=tim
		self.error_prev=error

		if output>0 :
			output=min(45,output)
		else:
			output=max(-45,output)

		return output

	
	def load_params(self):
		self.dist_tolerance     = float(rospy.get_param('/planner/dist_tolerance', 1.5))        #in metres ; default 5 metre
		self.bearing_tolerance  = float(rospy.get_param('/planner/bearing_tolerance', 4))    #in degrees ; default 10 degrees
		self.forward_max        = 45#float(rospy.get_param('/planner/forward_max', 40))          #in terms of pwm now
		self.forward_min        = 30#float(rospy.get_param('/planner/forward_min', 25))          #in terms of pwm now          #in terms of pwm value
		self.forward_mult       = float(rospy.get_param('/planner/forward_mult', 1.0))
		self.divider            = float(rospy.get_param('/planner/divider', 2))
		self.kp=-5
		self.ki=0
		self.kd=0

	def load_vars(self):
		self.state  = "run"  # states are 'run','pause','stop' 
		self.bearing_dest = 150
		self.bearing_curr = 0			#current bearing of the rover
		self.distance_to_dest = 20
		self.vel = 0
		self.omega = 0
		self.iter = 0

		self.error_prev=0
		self.error_int=0
		self.time_prev=0
		self.tsys=0
		
	def imuCallback(self,msg):
		self.bearing_curr = msg.yaw

	def goalCallback(self,msg):# each time i am getting a new goal i have to reset the distance calculator node
		self.distance_to_dest = float(msg.bearing)
		self.bearing_dest = float(msg.distance)

	def distCallback(self,msg): #getting the position of the bot from the pos calculator
		self.dist = msg.dist

	'''def rplCallback(self,msg): #getting the position of the bot from the pos calculator
		self.lidar = np.array(msg.ranges)'''

	def drive_pub(self): #used to send the drive node the info, the value of theta taken is 0 to 359 if any other value is given the service won't be called.
		rpm = WheelRpm()
		rpm.vel=self.vel
		rpm.omega=self.omega
		self.pub_drive.publish(rpm)

	def forward_vel_cal(self,vel_mult):
		vel = self.forward_min + (abs(self.forward_max-self.forward_min)*vel_mult)
		return min(vel,self.forward_max)

def signal_handler(signal, frame):  #For catching keyboard interrupt Ctrl+C
	print "\nProgram exiting....."
	plt.close()
	sys.exit(0)


if __name__ == '__main__':
	run = Planner()
	signal.signal(signal.SIGINT, signal_handler)
	run.spin()
