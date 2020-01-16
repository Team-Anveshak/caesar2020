#!/usr/bin/env python
#Plotting requires -- matplotlib , termcolor , cartopy , cython , scipy
import cartopy.crs as ccrs
import matplotlib.pyplot as plt
from termcolor import colored
import rospy,signal,sys,thread
from math import *
import time
from sensors.msg import Imu
from sensor_msgs.msg import NavSatFix

class Plot():

	def __init__(self):

		self.n = 0
		self.bearing_curr = float('nan')
		#define the projection of axes
		ax = plt.axes(projection=ccrs.PlateCarree())
		ax.stock_img()
		ax.coastlines()
		self.pt = plt.text(0,0, 'hello', fontsize=8,transform=ccrs.Geodetic()) 
		
		#array of way points
		file_path = "/home/swathi/aurora2019/src/navigation/config/gps_data.txt"
		try:
			self.f=open(file_path,'r')
			self.dest_lat_cont,self.dest_lon_cont = [],[]
			for l in self.f:
				row = l.split()
				self.dest_lat_cont.append(float(row[0]))
				self.dest_lon_cont.append(float(row[1]))
		except Exception,e:
			print colored("GPS data file not opened %s"%e,'red')
			sys.exit(0)
			
		'''self.dest_lat_cont,self.dest_lon_cont = [],[]
		print colored('\n Enter GPS way-points one by one in latitude<>longitude format \n', 'green')
		print colored('$  Type ok once done', 'white')
		l = raw_input('GPSprompt >>>')
		while (l != 'ok'):
			row = l.split()
			self.dest_lat_cont.append(row[0])
			self.dest_lon_cont.append(row[1])
			l = raw_input('GPSprompt >>>')'''
		
		#Plotting the waypoints	
		max_way_lat = float(max(self.dest_lat_cont));
		max_way_lon = float(max(self.dest_lon_cont));
		min_way_lat = float(min(self.dest_lat_cont));
		min_way_lon = float(min(self.dest_lon_cont));	
		for i in range(len(self.dest_lat_cont)):
			plt.plot(float(self.dest_lon_cont[i]),float(self.dest_lat_cont[i]),color='green', marker="$%d$"%(i+1), 				markersize=10, transform=ccrs.Geodetic(),)
		
		#Constraining the axes of the map
		ax.set_extent((min_way_lon-0.01,max_way_lon+0.01,min_way_lat-0.01,max_way_lat+0.01))	
	
	#Initialising node and subscriber
	def init_ros(self):
		try:
			rospy.init_node("cartopy_plotter")
		except Exception:
			print colored("\n $ Error initializing node @ cartopy_plotter \n",'red')	
		try:
			rospy.Subscriber("fix",NavSatFix, self.plotCallback)
		except Exception:
			print colored("$ Error opening subscriber @ fix",'red')
		rospy.Subscriber("imu",Imu, self.imu)


	#Callback function
	def plotCallback(self,msg):
		plt.plot(float(msg.longitude),float(msg.latitude),color='blue',
			 	markersize=3, marker='.', transform=ccrs.Geodetic(),) 	
		
		#Distance and Bearing Calculator
		lon1, lat1, lon2, lat2 = map(radians, [msg.longitude, msg.latitude,self.dest_lon_cont[self.n], 							self.dest_lat_cont[self.n]])
		dlon = lon2 - lon1
		dlat = lat2 - lat1
		a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
		c = 2 * atan2(sqrt(a), sqrt(1-a))
		dist_gps = 6371 * c*1000
		bearing = atan2(sin(lon2-lon1)*cos(lat2), (cos(lat1)*sin(lat2))-(sin(lat1)*cos(lat2)*cos(lon2-lon1)))
		bearing = degrees(bearing)
		
		#Displaying distance and bearing on map
		textstr = 'dist_gps(m)=%.5f\ndest_bearing(deg)=%.5f\ncurr_bearing(deg) = %.5f'%(dist_gps, bearing, self.bearing_curr)
		#textstr = 'dist_gps(m)=%.5f\nbearing_diff(deg) = %.5f\n'%(dist_gps, bearing-self.bearing_curr)
		self.pt.remove()
		self.pt = plt.text(self.dest_lon_cont[self.n], self.dest_lat_cont[self.n], 					textstr,fontsize=10,transform=ccrs.Geodetic())
		plt.pause(0.001)
		
	def imu(self,msg):
		self.bearing_curr = msg.yaw

	#Shifhting to next waypoint
	def key_intrp(self):
		print "\n Type ok when reached next point \n"
		flag = True
		while flag:
			text = raw_input('$GPS_plot >>>')
			if (text == 'ok'):
				if ( self.n < len(self.dest_lat_cont)-1 ):
					self.n = self.n+1
				else:
					print "All waypoints done"
					flag = False
	  		else:
  				print 'Invalid command'


	#For catching keyboard interrupt Ctrl+C
	def signal_handler(self,signal, frame):  
		print colored("\nProgram exiting.....\n",'red')
		sys.exit(0)
		
if __name__ == '__main__':
	x = raw_input("Do you want to start start plotting in cartopy? (y/n) : ")

	if(x == 'y'):
		xy = Plot()
		signal.signal(signal.SIGINT, xy.signal_handler)
		xy.init_ros()
		thread.start_new_thread(xy.key_intrp,()) 
		plt.show()
	else:
		print colored("Exiting.... \n",'red')
				
