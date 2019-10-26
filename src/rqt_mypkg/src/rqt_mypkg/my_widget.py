import os
import time
import thread
from thread import *
import subprocess
import rospy
import rospkg
import roslaunch
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
       
	rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        loadUi(ui_file, self, {'MyGraphicsView': MyGraphicsView})
	
        self.setObjectName('MyWidget')
	#QtCore.QMetaObject.connectSlotsByName()
	self.pushButton.clicked.connect(self.Joy_Arm)
        self.pushButton_2.clicked.connect(self.Kill_Joy_Arm)
        self.pushButton_3.clicked.connect(self.Joy_node)
        self.pushButton_5.clicked.connect(self.Drive)
        self.pushButton_6.clicked.connect(self.Rosserial)
        self.pushButton_7.clicked.connect(self.Autonomous)
        self.pushButton_9.clicked.connect(self.Kill_Joy_node)
        self.pushButton_10.clicked.connect(self.Kill_Drive)
        self.pushButton_11.clicked.connect(self.Kill_Rosserial)
        self.pushButton_13.clicked.connect(self.Exit_Autonomous)
        self.pushButton_14.clicked.connect(self.Arm)
        self.pushButton_15.clicked.connect(self.Stop_Arm)
	self.pushButton_8.clicked.connect(self.Rosserial_A)
        self.pushButton_12.clicked.connect(self.Kill_Rosserial_A)
	#start_new_thread(self.driveCallback,(ui_file))

    #def driveCallback(self,ui_file):
	#rq = rospkg.RosPack()
	#ui_file = os.path.join(rq.get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        #loadUi(ui_file, self, {'MyGraphicsView': MyGraphicsView})
	#while(True):
	#	rospy.Subscriber("/motion",WheelRpm) 
	#	rpm = WheelRpm()
	#	self.textEdit.setText(str(rpm.vel))
	#	print rpm.vel	
	#	time.sleep(1)
	

    def Joy_Arm(self):
        a=subprocess.Popen("./JOY_ARM.sh")

    def Kill_Joy_Arm(self):
        b=subprocess.Popen("./KILL_JOY_ARM.sh")

    def Joy_node(self):
        c=subprocess.Popen("./JOY_NODE.sh")
	#uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  	#roslaunch.configure_logging(uuid)
  	#launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/ashwanth/caesar2020/joy.launch'])
  	#launch.start()
    def Drive(self):
        d=subprocess.Popen("./DRIVE.sh")

    def Rosserial(self):
        e=subprocess.Popen("./ROSSERIAL_D.sh")

    def Autonomous(self):
        k=subprocess.Popen("./JOY_NODE.sh")
	
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
        f=subprocess.Popen("./KILL_JOY_NODE.sh")

    def Kill_Drive(self):
        g=subprocess.Popen("./KILL_DRIVE.sh")

    def Kill_Rosserial(self):
        h=subprocess.Popen("./KILL_ROSSERIAL_D.sh")

    def Exit_Autonomous(self):
        n=subprocess.Popen("./KILL_AUTONOMOUS.sh")

    def Kill_Rosserial_A(self):
        q=subprocess.Popen("./KILL_ROSSERIAL_A.sh")
    
    def Rosserial_A(self):
        r=subprocess.Popen("./Rosserial_A.sh")
 

    def Arm(self):
	o=subprocess.Popen("./ARM.sh")
	
    def Stop_Arm(self):
        p=subprocess.Popen("./KILL_ARM.sh")

	



