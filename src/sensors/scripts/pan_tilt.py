#!/usr/bin/env python

from sensors.msg import PanTilt
from sensor_msgs.msg import Joy
import rospy
import time
class Servo():

	def __init__(self):
		rospy.init_node("servoCtrl")
		rospy.Subscriber('/joy',Joy,self.joyCallback)
		self.servo_pub = rospy.Publisher('pan_tilt_ctrl',PanTilt, queue_size=10)
		self.panCtrl = 0
		self.tiltCtrl = 0
		self.panAngle = 120
		self.tiltAngle = 40
		self.relay = False
		self.lastTime = time.time()

	def joyCallback(self,msg):

		self.panCtrl = msg.axes[4]
		self.tiltCtrl = msg.axes[5]
		if msg.buttons[9]==1:
			self.relay = True
		elif msg.buttons[8] == 1:
			self.relay = False

	def main(self):
		servo_msg = PanTilt()
		servo_msg.rel = self.relay
		if time.time() - self.lastTime>0.05:
			if(self.panAngle>0 and self.panAngle<180):
				self.panAngle = self.panAngle + 2*self.panCtrl
			else:
				if self.panAngle<=0:
					self.panAngle = 1
				if self.panAngle>=180:
					self.panAngle = 179

			if(self.tiltAngle>0 and self.tiltAngle<180):
				self.tiltAngle = self.tiltAngle - 2*self.tiltCtrl
			else:
				if self.tiltAngle<=0:
					self.tiltAngle = 1
				if self.tiltAngle>=180:
					self.tiltAngle = 179

			servo_msg.pan = self.panAngle
			servo_msg.tilt = self.tiltAngle
			self.servo_pub.publish(servo_msg)
			self.lastTime = time.time()

	def spin(self):
		rate = rospy.Rate(1)
		while not rospy.is_shutdown():
				self.main()
		rate.sleep()

if __name__ == '__main__':
	servo = Servo()
	servo.spin()
