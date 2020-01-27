import os
import time
import thread
from thread import *
import subprocess
import rospy
import rospkg
import roslaunch
from sensor_msgs.msg import NavSatFix
from navigation.msg import Goal
from sensors.msg import Imu
from traversal.msg import WheelRpm
from subprocess import Popen, PIPE
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from PyQt5.QtWidgets import QFileDialog, QGraphicsView, QWidget
from PyQt5.QtCore import QThread

class MyGraphicsView(QGraphicsView):

    def __init__(self, parent=None):
        super(MyGraphicsView, self).__init__()

class MyWidget(QWidget):
   
    def __init__(self, context):
        
	
        super(MyWidget, self).__init__()
	self.lattitude=0
	self.longitude=0
	self.bearing = 0
	self.distance = 0
	self.vel = 0
	self.omega=0
	self.hb=0
	self.yaw=0
	self.pitch=0
	rp = rospkg.RosPack()
	ui_file = os.path.join(rp.get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        
        loadUi(ui_file, self, {'MyGraphicsView': MyGraphicsView})
	rospy.Subscriber("/motion",WheelRpm,self.driveCallback)
	rospy.Subscriber("/imu",Imu,self.imuCallback)
	rospy.Subscriber("/goal",Goal,self.goalCallback)
	rospy.Subscriber("/fix",NavSatFix,self.gpsCallback)
        self.setObjectName('MyWidget')
	
	self.pushButton.clicked.connect(self.Joy_Arm)
        self.pushButton_2.clicked.connect(self.Kill_Joy_Arm)
        self.pushButton_3.clicked.connect(self.Joy_node)
        self.pushButton_5.clicked.connect(self.Drive)
        self.pushButton_6.clicked.connect(self.Rosserial)
	self.pushButton_7.setCheckable(True)
	self.pushButton_7.pressed.connect(self.drivedisplay)
	self.pushButton_7.setAutoRepeatInterval(1000)
	self.pushButton_7.setAutoRepeat(True)
	if self.pushButton_7.autoRepeat():
		print 'auto'
        
	self.pushButton_9.clicked.connect(self.Kill_Joy_node)
        self.pushButton_10.clicked.connect(self.Kill_Drive)
        self.pushButton_11.clicked.connect(self.Kill_Rosserial)
        self.pushButton_13.clicked.connect(self.Exit_Autonomous)
	self.pushButton_17.clicked.connect(self.Autonomous)
        self.pushButton_14.clicked.connect(self.Arm)
        self.pushButton_15.clicked.connect(self.Stop_Arm)
	self.pushButton_8.clicked.connect(self.Rosserial_A)
        self.pushButton_12.clicked.connect(self.Kill_Rosserial_A)
	self.pushButton_4.clicked.connect(self.Rosnodes)
	self.pushButton_16.clicked.connect(self.Ping)	
	self.pushButton_19.clicked.connect(self.KillGPS)	
	self.pushButton_23.clicked.connect(self.KillIMU)	
	self.pushButton_18.clicked.connect(self.IMU)	
	self.pushButton_24.clicked.connect(self.GPS)	
	self.pushButton_20.clicked.connect(self.Ports)
	self.pushButton_22.clicked.connect(self.UpdateGPS)
	self.pushButton_21.clicked.connect(self.KillAll)
	self.pushButton_25.clicked.connect(self.PanTilt)
	self.pushButton_26.clicked.connect(self.Kill_Pantilt)
	
	
    

    def drivedisplay(self):
	text_1="vel="+str(self.vel)+"\n"+"omega="+str(self.omega)+"\n"+"hb="+str(self.hb) 
	self.lineEdit.setText(text_1)
	text_2="yaw="+str(self.yaw)+"\n"+"pitch="+str(self.pitch) +"\n"+ "lattitude="+str(self.lattitude)+"\n"+"longitude="+str(self.longitude) 
	self.textEdit.setText(text_2)
	text_3="bearing="+str(self.bearing)+"\n"+"distance="+str(self.distance) 
	self.lineEdit_2.setText(text_3)
	
    def driveCallback(self,msg):
	self.vel=msg.vel
	self.omega=msg.omega
	self.hb=msg.hb

    def imuCallback(self,msg):
	self.yaw=msg.yaw
	self.pitch=msg.pitch

    def goalCallback(self,msg):
	self.bearing=msg.bearing
	self.distance=msg.distance

    def gpsCallback(self,msg):
	self.lattitude=msg.latitude
	self.longitude=msg.longitude

    def Joy_Arm(self):
        a_0=subprocess.Popen("./JOY_ARM.sh")

    def Kill_Joy_Arm(self):
        a_1=subprocess.Popen("./KILL_JOY_ARM.sh")

    def Joy_node(self):
        a_2=subprocess.Popen("./JOY_NODE.sh")
	#uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  	#roslaunch.configure_logging(uuid)
  	#launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/ashwanth/caesar2020/joy.launch'])
  	#launch.start()
    def Drive(self):
        a_3=subprocess.Popen("./DRIVE.sh")

    def Rosserial(self):
        a_4=subprocess.Popen("./ROSSERIAL_D.sh")

    def Autonomous(self):
        a_5=subprocess.Popen("./AUTONOMOUS.sh")
	#stdouterr = os.popen4("./AUTONOMOUS.sh")[1].read()        
	#self.textEdit.setText(stdouterr)	
	#file1 = open("doc2.txt","r")
	#print file1.read() 
	#c="1"
	#a=""
	#while(c!=""):
	#	a=a+c
	#	c=file1.read()	
	#self.textEdit.setText(str(a))	
	#file1.close
	
	
	

    def Kill_Joy_node(self):
	a_6=subprocess.Popen("./KILL_JOY_NODE.sh")

    def Kill_Drive(self):
        a_7=subprocess.Popen("./KILL_DRIVE.sh")

    def Kill_Rosserial(self):
        a_8=subprocess.Popen("./KILL_ROSSERIAL_D.sh")

    def Exit_Autonomous(self):
        a_9=subprocess.Popen("./KILL_AUTONOMOUS.sh")
	
    def Kill_Rosserial_A(self):
        a_10=subprocess.Popen("./KILL_ROSSERIAL_A.sh")
    
    def Rosserial_A(self):
        a_11=subprocess.Popen("./Rosserial_A.sh")
 

    def Arm(self):
	a_12=subprocess.Popen("./ARM.sh")
	
    def Stop_Arm(self):
        a_13=subprocess.Popen("./KILL_ARM.sh")

    def IMU(self):
        a_14=subprocess.Popen("./IMU.sh")
    
    def GPS(self):
        a_15=subprocess.Popen("./GPS.sh")
    
    def KillIMU(self):
        a_16=subprocess.Popen("./KILL_IMU.sh")
    
    def Ports(self):
	#p_4=subprocess.Popen("./PORTS.sh")
	#cm = str("rosnode list")
	stdouterr = os.popen4("./PORTS.sh")[1].read()        
	self.textEdit.setText(stdouterr)
        

    def KillAll(self):
        a_17=subprocess.Popen("./KillAll.sh")

    def UpdateGPS(self):
        a_18=subprocess.Popen("./UPDATE_GPS.sh")
    
    def KillGPS(self):
        a_19=subprocess.Popen("./KILL_GPS.sh")

    def PanTilt(self):
        a_20=subprocess.Popen("./PANTILT.sh")

    def KillPantilt(self):
        a_21=subprocess.Popen("./KILL_PANTILT.sh")

    def Rosnodes(self):
	cm = str("rosnode list")
	stdouterr = os.popen4(cm)[1].read()        
	self.textEdit.setText(stdouterr)

    def Ping(self):
	ss=""
	sq=0
	process = Popen("ping 192.168.0.10", stdout=PIPE, shell=True)
    	while (sq<2):
       		line = process.stdout.readline().rstrip()
		ss=ss+line
		sq=sq+1        
	#if not line:
        #    break
	if(ss==""):
		ss="ERROR COULD NOT PING"
        self.textEdit.setText(ss)

	



