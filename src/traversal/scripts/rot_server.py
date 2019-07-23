#!/usr/bin/env python

import rospy
from man_ctrl.srv import *
from man_ctrl.msg import WheelRpm
from sensors.msg import Imu
import time

class rotateService():

	def __init__(self):

		rospy.init_node('rot_server')

		service = rospy.Service('rotator',rotate,self.rotator)

		self.pub_serv = rospy.Publisher("drive_inp",WheelRpm,queue_size = 10)

		rospy.Subscriber("imu", Imu, self.imuCallback)

		self.bearing_tolerance = rospy.get_param('/rot_server/bearing_tolerance',2)
		self.min_Omega = rospy.get_param('/rot_server/min_Omega',40) 
		self.max_Omega = rospy.get_param('/rot_server/max_Omega',100) 
		self.multiplier = rospy.get_param('/rot_server/bearing_tolerance',1)
		self.curr_bear = 0.0


	def spin(self):
		rate = rospy.Rate(5)
		while not rospy.is_shutdown():
			rate.sleep()
			rospy.spin()


	def rotator(self, request):

		Rpm = WheelRpm()

		self.final_bear = request.angle
		self.initial_bear = self.curr_bear
		self.remainAngle = self.final_bear - self.curr_bear
		self.omega = self.omegaManager(self.remainAngle)

		self.last_time = time.time()
		self.last_bear = self.curr_bear

		while (abs(self.remainAngle) > self.bearing_tolerance):

			Rpm.vel = 0
			self.remainAngle = self.final_bear - self.curr_bear
			self.omega = self.omegaManager(self.remainAngle)

			if self.remainAngle>180 :
				self.remainAngle = self.remainAngle - 360
			elif self.remainAngle<-180 :
				self.remainAngle = self.remainAngle + 360
				
			if self.remainAngle<0:
				Rpm.omega = int(self.omega)
			else:
				Rpm.omega = int(-self.omega)

			self.pub_serv.publish(Rpm)

		Rpm = WheelRpm()
		self.pub_serv.publish(Rpm)

		return rotateResponse("Rotate_finished")


	def omegaManager(self,angle):
		Omega = self.min_Omega + abs(angle)*self.multiplier				#units in rpm giving a min of 10rpm
		if Omega>self.max_Omega:
			Omega = self.max_Omega
		return Omega

	def imuCallback(self,msg):
		self.curr_bear=msg.yaw


if __name__ == '__main__':
	run = rotateService()
	run.spin()