#!/usr/bin/env python
import rospy
from traversal.msg import WheelRpm
from detection.msg import detection
from detection.srv import *
import os
import subprocess, signal

#coord_array=np.zeros(10)
def get_number(line,n):
	a=(int(line[(4*(n-1))+2])*100)+(int(line[((4*n)-1)])*10)+int(line[((4*n))])
	return a-100
		
class detect():

	def __init__(self):
		rospy.init_node('search_control',anonymous=False)
		print "search_control initialized"
		self.reach = rospy.Service('reached', reached ,self.detect)
		self.pub=rospy.Publisher('drive_inp',WheelRpm,queue_size=10)
		self.rate1=rospy.Rate(3)
		self.rate2=rospy.Rate(0.5)
		self.rpm = WheelRpm()
		
		f=open('/home/anveshak/caesar2020/src/detection/save_time.txt',"w+")
		f.write("b")
		f.close()  
		os.system('bash ~/run.sh >/dev/null &')
	
	def detect(self,msg):
		i=0
		print "detect started"
		f=open('/home/anveshak/caesar2020/src/detection/save_time.txt',"w+")
		f.write("a")
		f.close()
		
		
		
		while True:
			g=open('/home/anveshak/caesar2020/src/detection/data.txt',"r")
			lines = g.readlines()
			last_line = lines[-1]
			g.close()
		
			if int(last_line[0])==0 or int(last_line[0])==1 or int(last_line[0])==2:
				print "no ball detected" 
				self.rpm.vel=0
				self.rpm.omega=10
				self.rpm.hb=False
				
				self.pub.publish(self.rpm)
				self.rate1.sleep()
			else: 
				i=0
				while (i<=2):
					g=open('/home/anveshak/caesar2020/src/detection/data.txt',"r")
					lines = g.readlines()
					last_line = lines[-1]
		
					if int(last_line[0])==0 or int(last_line[0])==1 or int(last_line[0])==2:
						print "no ball detected" 
						self.rpm.vel=0
						self.rpm.omega=0
						self.rpm.hb=False
						i=i+1	
						print i
						self.pub.publish(self.rpm)
						g.close()
		
					else :
						i=0
						xmin=get_number(last_line,1)
						ymin=get_number(last_line,2)
						xmax=get_number(last_line,3)
						ymax=get_number(last_line,4)
				
						box_height=ymax-ymin
						box_width=xmax-xmin
						print("bbh=",box_height)
						print("bbw=",box_width)
						
						
						thresh1=160
						thresh2=480
		
							
					
						x=int((xmin+xmax)/2)
						
							
		
						if box_height>=40 or box_width>=40:
							self.rpm.vel=0
							self.rpm.omega=0
							self.rpm.hb=False
							f=open('/home/anveshak/caesar2020/src/detection/save_time.txt',"w+")
							f.write("b")
							f.close()
							print("ball found and reached")
							#os.system("pkill python3")
							i=10
							
		
						else:
					
							
							if x<thresh1:
								self.rpm.vel=0
								self.rpm.omega=10
								self.rpm.hb=False
							elif x>thresh2:
								self.rpm.vel=0
								self.rpm.omega=-10
								self.rpm.hb=False
		
							else :
								self.rpm.vel=15
								self.rpm.omega=0
								self.rpm.hb=False
				
					self.pub.publish(self.rpm)
					g.close()
					self.rate2.sleep()

				if i==10:
					break

		g=open('/home/anveshak/caesar2020/src/detection/data.txt',"w+")
		g.write('0 0 0 0 0 0.000000')
		g.close()
		return "Detection done"

	def spin(self):
		while True:
			pass

def signal_handler(signal, frame):  #For catching keyboard interrupt Ctrl+C
	print "\nProgram exiting....."
	os.system("pkill python3")
	sys.exit(0)
				
	
if __name__=='__main__':
	det=detect()
	signal.signal(signal.SIGINT, signal_handler)
	det.spin()
	
		
			

