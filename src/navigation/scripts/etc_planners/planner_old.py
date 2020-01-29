#!/usr/bin/env python
#add service to reload params
import rospy
from navigation.srv import *
from man_ctrl.srv import *
from navigation.msg import *
from sensors.msg import *
from man_ctrl.msg import *

import thread, time
from termcolor import colored
import numpy as np

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

		#service clients
		rospy.wait_for_service('rotator')
		try:
			self.drive_rotate_srv = rospy.ServiceProxy('rotator', rotate)
		except Exception,e:
			print "Service call failed: %s"%e

		self.bearing_dest = self.bearing_curr        

	def spin(self):
		rate = rospy.Rate(1)
		self.bearing_dest = self.bearing_curr
		while not rospy.is_shutdown():
			self.main() #main func
			rate.sleep()

	def main(self):
		#print(self.state)
		if(self.state=="run"):
				if(self.distance_to_dest > self.dist_tolerance): 
					self.pub_planner_state.publish(0)
					if(abs(self.bearing_dest-self.bearing_curr)<self.bearing_tolerance):
						forward_vel = self.forward_vel_cal(self.forward_min,self.forward_max,1.5)
						self.drive_pub(forward_vel,0)  #setup a primitive pid w.r.t to diatnce to be travelled.
					else:
						self.obs_scanner_active = False
						try:
							print colored('\n Sending request for %f'%self.bearing_dest,'white')
							result = self.drive_rotate_srv(float(self.bearing_dest))
							print result
						except rospy.ServiceException,e :
							print "Service call failed: %s"%e
						#send service call to drive node to turn to self.bearing destination
				else:
					self.pub_planner_state.publish(1)
					self.drive_pub(0.0,0.0)
					self.obs_scanner_active = False
					rospy.loginfo("Destination reached")

		elif(self.state=="pause"):
			pass
			
		elif(self.state=="stop"):
			self.drive_pub(0.0,0.0,self.forward_max)
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
		# print(srv_msg.contin)
		return plan_stateResponse(self.state)

	def load_params(self):
		self.dist_tolerance     = float(rospy.get_param('/planner/dist_tolerance', 1.5))        #in metres ; default 5 metre
		self.bearing_tolerance  = float(rospy.get_param('/planner/bearing_tolerance', 11.0))    #in degrees ; default 10 degrees
		self.forward_max        = float(rospy.get_param('/planner/forward_max', 18.0))          #in terms of pwm now
		self.forward_min        = float(rospy.get_param('/planner/forward_min', 9.0))          #in terms of pwm now          #in terms of pwm value
		self.forward_mult       = float(rospy.get_param('/planner/forward_mult', 1.0))

	def load_vars(self):
		self.state                  = "stop"  # states are 'run','pause','stop'
		self.distance_to_dest       = 400.0
		self.bearing_dest           = 0.0
		self.bearing_curr           = 0.0   #current bearing of the rover

	def imuCallback(self,msg):
		self.bearing_curr = msg.yaw

	def goalCallback(self,msg):# each time i am getting a new goal i have to reset the distance calculator node
		self.distance_to_dest = msg.distance
		self.bearing_dest = msg.bearing

	def distCallback(self,msg): #getting the position of the bot from the pos calculator
		self.dist = msg.dist

	def rplCallback(self,msg): #getting the position of the bot from the pos calculator
		self.lidar = np.array(msg.ranges)

	def reset(self): #for resetting all variables to start position, sending the distance calculator to reset etc
		self.load_vars()
		self.load_params()
		self.drive_pub(0.0,0.0)
		#need to reset distance calculator

	def drive_pub(self,vel,omega): #used to send the drive node the info, the value of theta taken is 0 to 359 if any other value is given the service won't be called.
		rpm =WheelRpm()
		rpm.vel=vel
		rpm.omega=omega
		self.pub_drive.publish(rpm)

	def forward_vel_cal(self,vel_min,vel_max,vel_mult):
		vel = vel_min + (abs(vel_max-vel_min)*vel_mult)
		return min(vel,vel_max)


if __name__ == '__main__':
	run = Planner()
	run.spin()
