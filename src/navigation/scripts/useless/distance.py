#!/usr/bin/env python
import rospy
from navigation.msg import * 
from man_ctrl.msg import * #Pos

class Dist():

	def __init__ (self):
		rospy.init_node("Distance calculator")
		
		#Subscribers
		try:
		 	rospy.Subscriber("pos",Pos,self.posCallback)
		except Exception,e:
			print e

		#service server
		self.dist_ser = rospy.Service('Distance_reset',dist_state,self.dist_ctrl)

		#publishers
		self.pub_dist = rospy.Publisher("curr_pos",Enc_dist,queue_size=10)

		self.curr_dist = 0.0

	def spin(self):
		rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			self.pub_dist(self.curr_dist)
			rate.sleep()

	def posCallback(self,msg): #getting the position of the bot from the pos calculator
		self.curr_dist += msg.data

	def dist_ctrl(self,srv_msg):
		self.curr_dist = 0.0
		return dist_stateResponse('Distance node reset done !!')

if __name__ == '__main__':
	run = Dist()
	run.spin()

