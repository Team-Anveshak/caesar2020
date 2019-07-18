#!/usr/bin/env python
#add option to load data from file or enter new
import rospy
from sensor_msgs.msg import NavSatFix
from navigation.msg import Goal,Planner_state
from navigation.srv import *
#from obj_detect.srv import *

from math import *
import sys, signal,thread
from termcolor import colored


class GPS() :
	def __init__(self):
		rospy.init_node("gdm")

		rospy.wait_for_service('Planner_state_ctrl')
		self.state_srv = rospy.ServiceProxy('Planner_state_ctrl', plan_state)
		
		'''rospy.wait_for_service('obj_detect')
		self.obj_srv = rospy.ServiceProxy('obj_detect', obj_detect)'''

		self.pub_goal = rospy.Publisher('goal', Goal,queue_size=10) 	#Publisher to planner

		rospy.Subscriber("fix", NavSatFix, self.gpsCallback) 		#From nmea node
		rospy.Subscriber("planner_state",Planner_state, self.plannerCallback)

		if(raw_input('Do you want to load from file [y/n] ?') == 'y'):
			file_path = "/home/anveshak/caesar2020/src/navigation/config/gps_data.txt"
			try:
				self.f=open(file_path,'r')
				self.dest_lat_cont,self.dest_lon_cont = [],[]
				for l in self.f:
					row = l.split()
					self.dest_lat_cont.append(row[0])
					self.dest_lon_cont.append(row[1])
			except Exception:
				print colored("GPS data file not opened",'red')
				sys.exit(0)
		else:
			self.dest_lat_cont,self.dest_lon_cont = [],[]			#array of way points
			print "Enter GPS way-points one by one in latitude<>longitude format"
			print colored('$ Type ok once done', 'green')
			l = raw_input('GPSprompt >>>')
			while (l != 'ok'):
				row = l.split()
				self.dest_lat_cont.append(row[0])
				self.dest_lon_cont.append(row[1])
				l = raw_input('GPSprompt >>>')

		self.curr_lat = 0.0
		self.curr_lon = 0.0
		self.bearing = 0
		self.planner_status = 0
		self.distance = 0
		thread.start_new_thread(self.key_intrp,())


	def run(self):

		goal = Goal()
		flag = True
		self.srv("rst")

		while (not rospy.is_shutdown()) and flag :

			for i in range(len(self.dest_lat_cont)):

				self.dest_lat= float(self.dest_lat_cont[i])
				self.dest_lon= float(self.dest_lon_cont[i])
				self.dist_gps,self.bearing=self.cal()

				goal.distance = self.dist_gps
				goal.bearing = self.bearing
				self.pub_goal.publish(goal)

				self.srv("contin")

				while(self.planner_status == 0):
					try:
						self.dist_gps,self.bearing=self.cal()
						goal.distance = self.dist_gps
						goal.bearing = self.bearing
						self.pub_goal.publish(goal)
						rate = rospy.Rate(1)
						rate.sleep()
					except Exception:
						print colored("ERROR in planner part",'red')

				if(self.planner_status == 1):
					self.srv("rst")
					rate = rospy.Rate(1)
					rate.sleep()
					self.srv("pause")
				
				'''try:
					result = self.obj_srv()
					print "Object detection: \n%s"%resp
				except rospy.ServiceException:
					print colored("Object service failed",'red')'''

				print colored("\n Moving to next GPS point.. \n",'white')

			print colored('Successfully past all waypoints!!','white')
			flag = False


	def srv(self,arg):
		temp = plan_stateRequest()
		if(arg == "pause"):
			#print "Pausing...."
			temp.pause = 1
		elif(arg == "contin"):
			#print "Continuing..."
			temp.contin = 1
		elif(arg == "rst"):
			#print "Resetting..."
			temp.rst = 1
		else:
			return 0

		try:
			resp = self.state_srv(temp)
			return resp
		except rospy.ServiceException:
			colored('ERROR calling planner service','red')
			return 'Error'

		print colored("\n Type 'p' to pause ----- 'c' to contin ----- 'r' to rst the Planner \n", 'blue')


	def cal(self): #distance and bearing calculator
		lon1, lat1, lon2, lat2 = map(radians, [self.curr_lon, self.curr_lat, self.dest_lon, self.dest_lat])
		dlon = lon2 - lon1
		dlat = lat2 - lat1
		a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
		c = 2 * atan2(sqrt(a), sqrt(1-a))
		dist_gps = 6371 * c*1000
		bearing = atan2(sin(lon2-lon1)*cos(lat2), (cos(lat1)*sin(lat2))-(sin(lat1)*cos(lat2)*cos(lon2-lon1)))
		bearing = degrees(bearing)
		return dist_gps,bearing

	def gpsCallback(self,msg):
		self.curr_lat = msg.latitude
		self.curr_lon = msg.longitude
		self.status = msg.status

	def plannerCallback(self,msg):
		self.planner_status = msg.status

	def key_intrp(self):
		print colored("\n Type 'p' to pause ----- 'c' to contin ----- 'r' to rst the Planner \n", 'blue')

		while True:
			text = raw_input(colored('$GPS_node >>> ','white'))
			if (text == 'p'):
				self.srv("pause")
			elif (text == 'c'):
				self.srv("contin")
			elif (text == 'r'):
				self.srv("rst")
			else:
				print 'Invalid command'


def signal_handler(signal, frame):  #For catching keyboard interrupt Ctrl+C
	print "\nProgram exiting....."
	sys.exit(0)


if __name__ == '__main__':

	x = raw_input('Do you want to start the node? (y/n) : ')

	gps = GPS()
	signal.signal(signal.SIGINT, signal_handler)

	if(x == 'y'):
		gps.run()
	else:
		sys.exit()
