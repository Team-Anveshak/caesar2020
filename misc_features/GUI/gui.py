import subprocess
import sys
import time
import os
from subprocess import Popen, PIPE
from PyQt5 import QtCore, QtGui, QtWidgets


class MainDialog(object):

    def __init__(self):
	super(MainDialog, self).__init__()
        Dialog.setObjectName("Dialog")
        Dialog.resize(725, 530)
        self.pushButton = QtWidgets.QPushButton(Dialog)
        self.pushButton.setGeometry(QtCore.QRect(40, 50, 141, 41))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QtWidgets.QPushButton(Dialog)
        self.pushButton_2.setGeometry(QtCore.QRect(190, 50, 151, 41))
        self.pushButton_2.setFlat(False)
        self.pushButton_2.setObjectName("pushButton_2")
        self.groupBox = QtWidgets.QGroupBox(Dialog)
        self.groupBox.setGeometry(QtCore.QRect(30, 110, 321, 201))
        self.groupBox.setObjectName("groupBox")
        self.pushButton_3 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_3.setGeometry(QtCore.QRect(10, 20, 141, 27))
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_9 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_9.setGeometry(QtCore.QRect(160, 20, 151, 27))
        self.pushButton_9.setObjectName("pushButton_9")
        self.pushButton_5 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_5.setGeometry(QtCore.QRect(10, 50, 141, 27))
        self.pushButton_5.setObjectName("pushButton_5")
        self.pushButton_6 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_6.setGeometry(QtCore.QRect(10, 80, 141, 27))
        self.pushButton_6.setObjectName("pushButton_6")
        self.pushButton_10 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_10.setGeometry(QtCore.QRect(160, 50, 151, 27))
        self.pushButton_10.setObjectName("pushButton_10")
        self.pushButton_11 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_11.setGeometry(QtCore.QRect(160, 80, 151, 27))
        self.pushButton_11.setObjectName("pushButton_11")
        self.pushButton_14 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_14.setGeometry(QtCore.QRect(10, 110, 141, 27))
        self.pushButton_14.setObjectName("pushButton_14")
        self.pushButton_7 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_7.setGeometry(QtCore.QRect(10, 140, 141, 27))
        self.pushButton_7.setObjectName("pushButton_7")
        self.pushButton_8 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_8.setGeometry(QtCore.QRect(10, 170, 141, 27))
        self.pushButton_8.setObjectName("pushButton_8")
        self.pushButton_12 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_12.setGeometry(QtCore.QRect(160, 170, 151, 27))
        self.pushButton_12.setObjectName("pushButton_12")
        self.pushButton_13 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_13.setGeometry(QtCore.QRect(160, 140, 151, 27))
        self.pushButton_13.setObjectName("pushButton_13")
        self.pushButton_15 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_15.setGeometry(QtCore.QRect(160, 110, 151, 27))
        self.pushButton_15.setObjectName("pushButton_15")
        self.textEdit_2 = QtWidgets.QTextEdit(Dialog)
        self.textEdit_2.setGeometry(QtCore.QRect(360, 130, 291, 81))
        self.textEdit_2.setObjectName("textEdit_2")
        self.groupBox_2 = QtWidgets.QGroupBox(Dialog)
        self.groupBox_2.setGeometry(QtCore.QRect(30, 330, 321, 80))
        self.groupBox_2.setObjectName("groupBox_2")
        self.pushButton_4 = QtWidgets.QPushButton(self.groupBox_2)
        self.pushButton_4.setGeometry(QtCore.QRect(10, 20, 301, 27))
        self.pushButton_4.setObjectName("pushButton_4")
        self.pushButton_16 = QtWidgets.QPushButton(self.groupBox_2)
        self.pushButton_16.setGeometry(QtCore.QRect(10, 50, 301, 27))
        self.pushButton_16.setObjectName("pushButton_16")
        self.textEdit_3 = QtWidgets.QTextEdit(Dialog)
        self.textEdit_3.setGeometry(QtCore.QRect(360, 250, 291, 121))
        self.textEdit_3.setObjectName("textEdit_3")
        self.pushButton_17 = QtWidgets.QPushButton(Dialog)
        self.pushButton_17.setGeometry(QtCore.QRect(550, 100, 99, 27))
        self.pushButton_17.setObjectName("pushButton_17")
        self.pushButton_18 = QtWidgets.QPushButton(Dialog)
        self.pushButton_18.setGeometry(QtCore.QRect(560, 220, 99, 27))
        self.pushButton_18.setObjectName("pushButton_18")
        self.pushButton_19 = QtWidgets.QPushButton(Dialog)
        self.pushButton_19.setGeometry(QtCore.QRect(560, 380, 99, 27))
        self.pushButton_19.setObjectName("pushButton_19")
        self.pushButton_20 = QtWidgets.QPushButton(Dialog)
        self.pushButton_20.setGeometry(QtCore.QRect(560, 500, 99, 27))
        self.pushButton_20.setObjectName("pushButton_20")
        self.lineEdit = QtWidgets.QLineEdit(Dialog)
        self.lineEdit.setGeometry(QtCore.QRect(360, 20, 291, 71))
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit_2 = QtWidgets.QLineEdit(Dialog)
        self.lineEdit_2.setGeometry(QtCore.QRect(360, 410, 301, 81))
        self.lineEdit_2.setObjectName("lineEdit_2")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)


    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.pushButton.setText(_translate("Dialog", "Roscore"))
        self.pushButton_2.setText(_translate("Dialog", "Kill Roscore"))
        self.groupBox.setTitle(_translate("Dialog", "Nodes"))
        self.pushButton_3.setText(_translate("Dialog", "Joy_node"))
        self.pushButton_9.setText(_translate("Dialog", "Kill Joy_node"))
        self.pushButton_5.setText(_translate("Dialog", "Drive"))
        self.pushButton_6.setText(_translate("Dialog", "Rosserial"))
        self.pushButton_10.setText(_translate("Dialog", "Kill drive"))
        self.pushButton_11.setText(_translate("Dialog", "Kill Rosserial"))
        self.pushButton_14.setText(_translate("Dialog", "Arm"))
        self.pushButton_7.setText(_translate("Dialog", "Autonomous"))
        self.pushButton_8.setText(_translate("Dialog", "Plotting_node"))
        self.pushButton_12.setText(_translate("Dialog", "Close Plotting_node"))
        self.pushButton_13.setText(_translate("Dialog", "Exit Autonomous"))
        self.pushButton_15.setText(_translate("Dialog", "Stop Arm"))
        
        self.groupBox_2.setTitle(_translate("Dialog", "Debugging"))
        self.pushButton_4.setText(_translate("Dialog", "Rqt_logger_level"))
        self.pushButton_16.setText(_translate("Dialog", "Rqt_console"))
        self.pushButton_17.setText(_translate("Dialog", "GPS"))
        self.pushButton_18.setText(_translate("Dialog", "Motion"))
        self.pushButton_19.setText(_translate("Dialog", "Rosnodes"))
        self.pushButton_20.setText(_translate("Dialog", "Spare"))
	self.pushButton.clicked.connect(self.Roscore)
        self.pushButton_2.clicked.connect(self.Kill_Roscore)
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
        self.pushButton_4.clicked.connect(self.Rqt_logger_level)
        self.pushButton_16.clicked.connect(self.Rqt_console)
	#self.pushButton_17.clicked.connect(self.GPS)
	self.pushButton_18.clicked.connect(self.Motion)
	self.pushButton_19.clicked.connect(self.Rosnodes)
	#self.pushButton_20.clicked.connect(self.Spare)
    def Roscore(self):
        a=subprocess.Popen("./ROSCORE.sh")

    def Kill_Roscore(self):
        b=subprocess.Popen("./KILL_ROSCORE.sh")

    def Joy_node(self):
        c=subprocess.Popen("./JOY_NODE.sh")

    def Drive(self):
        d=subprocess.Popen("./DRIVE.sh")

    def Rosserial(self):
        e=subprocess.Popen("./ROSSERIAL.sh")

    def Autonomous(self):
        k=subprocess.Popen("./AUTONOMOUS.sh")

    def Kill_Joy_node(self):
        f=subprocess.Popen("./KILL_JOY_NODE.sh")

    def Kill_Drive(self):
        g=subprocess.Popen("./KILL_DRIVE.sh")

    def Kill_Rosserial(self):
        h=subprocess.Popen("./KILL_ROSSERIAL.sh")

    def Exit_Autonomous(self):
        n=subprocess.Popen("./KILL_AUTONOMOUS.sh")

    def Arm(self):
        #o=subprocess.check_output(['python','drive.py','-i','test.txt'])
	o=subprocess.Popen("./ARM.sh")
	
    def Stop_Arm(self):
        p=subprocess.Popen("./KILL_ARM.sh")

    def Rqt_logger_level(self):
        i=subprocess.Popen("./LOGGER_LEVEL.sh")

    def Rqt_console(self):
        j=subprocess.Popen("./CONSOLE.sh")
    #def GPS(self):
    def Rosnodes(self):
	cm = str("export ROS_MASTER_URI=http://192.168.0.10:11311/;export ROS_IP=192.168.0.2;rosnode list")
        #stdouterr = os.popen(cmd[,r[,0]])
	stdouterr = os.popen4(cm)[1].read()        
	self.textEdit_3.setText(stdouterr)
    def Motion(self):
	cmd = str("export ROS_MASTER_URI=http://192.168.0.10:11311/;export ROS_IP=192.168.0.2;rostopic echo /motion")
	process = Popen(cmd, stdout=PIPE, shell=True)
	motion = ""
	x=3
    	while (x>0):
        	line = process.stdout.readline().rstrip()
        	if not line:
        		break
		motion = motion + line
		x=x-1

        self.textEdit_2.setText(motion)

    #def Spare(self):


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = MainDialog()
    Dialog.show()
    sys.exit(app.exec_())
