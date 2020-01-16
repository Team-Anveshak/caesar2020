#!/usr/bin/env python
import rospy
from science.msg import *
from sensor_msgs.msg import Joy
import numpy
import math

class drive():

	def __init__(self):

		rospy.init_node("drive")
		self.pub_actuate = rospy.Publisher("motion_sci",actuators,queue_size=10)
		rospy.Subscriber("/joy",Joy,self.joyCallback)
		rospy.Subscriber("/s_val",sensor,self.sensorCallback)
		self.tsl_full = 0
		self.lps_temp = 0
		self.lps_pressure = 0
		self.sht_humidity = 0
		self.sht_temp = 0
		self.stepper = 0
		self.linac = 0
		self.rhino = 0
		self.micro = 0
		self.lid = 0
		self.sht = 0

	def spin(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.main()
			rate.sleep()

	def main(self):
            act=actuators()
            act.stepper=self.stepper
            act.linac=self.linac
            act.micro=self.micro
            act.rhino=self.rhino
            act.lid=self.lid	
	    act.sht=self.sht
            self.pub_actuate.publish(act)

	def joyCallback(self,msg):
		
		if(msg.axes[1]>0.05):
			self.linac=1
		elif(msg.axes[1]<-0.05):
			self.linac=-1
		else:
			self.linac=0
		if(msg.axes[3]>0.05):
			self.rhino=1
		elif(msg.axes[3]<-0.05):
			self.rhino=-1
		else:
			self.rhino=0
		if(msg.axes[4]>0.05):
			self.micro=1
		elif(msg.axes[4]<-0.05):
			self.micro=-1
		else:
			self.micro=0
		if(msg.buttons[0]==1):
			self.lid=1
		elif(msg.buttons[2]==1):
			self.lid=-1
		else:
			self.lid=0
		if(msg.buttons[4]==1):
			self.stepper=1
		elif(msg.buttons[6]==1):
			self.stepper=-1
		else:
			self.stepper=0
		if(msg.buttons[5]==1):
			self.sht=1
		else:
			self.sht=0
			
	def sensorCallback(self,msg):
		self.tsl_full = msg.tsl_full
		self.lps_temp = msg.lps_temp
		self.lps_pressure = msg.lps_pressure
		self.sht_humidity = msg.sht_humidity
		self.sht_temp = msg.sht_temp
		print "LUMINOSITY=" + str(self.tsl_full)
		print "ATMOSPHERIC TEMPERATURE=" + str(self.lps_temp)
		print "PRESSURE=" + str(self.lps_pressure)
		print "SOIL HUMIDITY=" + str(self.sht_humidity)
		print "SOIL TEMPERATURE=" + str(self.sht_temp)
		print "***"


if __name__ == '__main__':
	run = drive()
	run.spin()	
