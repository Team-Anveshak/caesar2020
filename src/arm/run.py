#!/usr/bin/env python

import rospy
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs

class Node:
    def __init__(self):
        
        self.outbuff = [0] * 4
        
        self.pub = rospy.Publisher('stm_write', std_msgs.Int32MultiArray, queue_size=10)
        rospy.init_node('arm_drive')
        rospy.Subscriber('joy', sensor_msgs.Joy, self.joyCallback)
    
    def joyCallback_0 (self, msg):
        self.outbuff = [ int (msg.axes[i] * 0xFF) for i in range(4) ]
        self.outbuff += [ (msg.buttons[i] - msg.buttons[i+2]) * 0xFF for i in range(4, 6) ]
        print (self.outbuff)

    def joyCallback (self, msg):
        outbuff = [0, 0, 0, 0, 0, 0]
        
        outbuff = [ int (msg.axes[i] * 0xFF) for i in range(4) ]
        outbuff += [ (msg.buttons[i] - msg.buttons[i+2]) * 0xFF for i in range(4, 6) ]
        
        axes = [ int (msg.axes[i] * 0xFF) for i in range(4) ]
        buttons = [ (msg.buttons[i] - msg.buttons[i+2]) * 0xFF for i in range(4, 6) ]
        
        outbuff[0] = - axes[0]
        outbuff[1] = axes[1]
        outbuff[2] = axes[3]
        outbuff[3] = buttons[1]
        outbuff[4] = buttons[0]
        outbuff[5] = axes[2]
        
        self.outbuff = outbuff
        print (self.outbuff)

    def run (self):
        rate = rospy.Rate (50)
        while not rospy.is_shutdown():
            rate.sleep()
            msg = self.createMsg (self.outbuff)
            self.pub.publish (msg)
    
    def createMsg (self, buff):
        msg = std_msgs.Int32MultiArray()
        msg.data = buff[:]
        
        msg.layout = std_msgs.MultiArrayLayout()
        msg.layout.data_offset = 0
        
        msg.layout.dim = [ std_msgs.MultiArrayDimension() ]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
        msg.layout.dim[0].label = 'write'
        
        return msg

node = Node()
node.run()

