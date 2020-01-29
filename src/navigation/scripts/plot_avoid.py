#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib as mpt
import numpy as np

import rospy
import navigation.msg
import std_msgs.msg as std_msgs

from rospy.numpy_msg import numpy_msg

import threading
import time

class Node:
    def __init__ (self):
        self.obs_data = np.zeros(0)
        self.val_data = np.zeros(0, dtype=np.bool_)
        self.dist, self.deviation, self.goal_dist = 0, 0, np.nan
        
        self.obs_lock = threading.Lock()
        self.val_lock = threading.Lock()
        self.tar_lock = threading.Lock()
        
        rospy.Subscriber ('obs_map', numpy_msg(navigation.msg.ObstacleMap), self.obs_map_cb)
        rospy.Subscriber ('valid_map', numpy_msg(std_msgs.ByteMultiArray), self.valid_map_cb)
        rospy.Subscriber ('target_loc', navigation.msg.Target, self.target_cb)
        
        rospy.init_node ('plot_avoid')
    
    def obs_map_cb (self, msg):
        with self.obs_lock:
            self.obs_data = msg.data
    
    def valid_map_cb (self, msg):
        with self.val_lock:
            self.val_data = msg.data * 9
    
    def target_cb (self, msg):
        with self.tar_lock:
            self.dist, self.deviation, self.goal_dist = msg.target_dist, msg.deviation, msg.goal_dist
            self.deviation = np.radians(self.deviation)
    
    def run (self):
    
        with self.obs_lock, self.val_lock:
            obs_rad = np.linspace (0, 2*np.pi, self.obs_data.size, endpoint=False)
            val_rad = np.linspace (0, 2*np.pi, self.val_data.size, endpoint=False)
        
            plt.ion()
            fig = plt.figure()
            ax = fig.add_subplot (111, polar=True)
            
            obs_lin, = ax.plot (obs_rad, self.obs_data, marker='o')
            val_lin, = ax.plot (val_rad, self.val_data, marker='x')
            
            val_plgn = None
            try:
                val_plgn, = ax.fill (val_rad, self.val_data, color='orange')
            except IndexError:
                pass
            #print len (val_bars), type (val_bars)
            
            tar_lin, = ax.plot (self.deviation, self.dist, marker='^', markersize=15)
        
        while not rospy.is_shutdown():
            with self.obs_lock, self.val_lock:
                obs_rad = np.linspace (0, 2*np.pi, self.obs_data.size, endpoint=False)
                val_rad = np.linspace (0, 2*np.pi, self.val_data.size, endpoint=False)
                val_lin.set_xdata (val_rad)
                obs_lin.set_xdata (obs_rad)
                
                try:
                    ax.set_ylim([0, 10])
                except: pass
                val_lin.set_ydata (self.val_data)
                obs_lin.set_ydata (self.obs_data)
                
                try:
                    val_plgn.remove ()
                except: pass
                try:
                    val_plgn, = ax.fill (val_rad, self.val_data, color='orange')
                except IndexError:
                    pass
            
            with self.tar_lock:
                tar_lin.set_xdata (self.deviation)
                tar_lin.set_ydata (self.dist * 0.9)
            
            try:
                fig.canvas.draw()
                fig.canvas.flush_events()
            except:
                break
                
            time.sleep (0.1)
            
        print 'done'

def run ():
    node = Node()
    node.run()
    
run()
