#!/usr/bin/env python

'''
Reads from the Zed ROS node, publishes an ObstacleMap.msg
PointCloud2 x, y, z, rgb
x seems to be depth

'''

import numpy as np
import scipy.interpolate

import rospy
import sensor_msgs.msg as sensor_msgs
import navigation.msg

from rospy.numpy_msg import numpy_msg

import sys

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import threading
import time

class Node:
    def __init__ (self):
        self.data = np.empty ((1, 4), dtype=np.float32)
        self.lock = threading.Lock()
    
        rospy.init_node ('zed_interpret')
        self.sub = rospy.Subscriber ('zed/zed_node/point_cloud/cloud_registered', sensor_msgs.PointCloud2, self.callback, queue_size=1)
        self.pub = rospy.Publisher ('obs_map', numpy_msg(navigation.msg.ObstacleMap), queue_size=10)
    
    def moving_average (self, a, n):
        if (a.size + 1 < n):
            return a
    
        ret = np.cumsum(a.filled(0))
        ret[n:] = ret[n:] - ret[:-n]
        counts = np.cumsum(~a.mask)
        counts[n:] = counts[n:] - counts[:-n]
        ret[~a.mask] /= counts[~a.mask]
        ret[a.mask] = np.nan

        return ret
    
    def callback (self, msg):
        self.lock.acquire()
        
        MAX_SLOPE = 1
        MIN_SLOPE = -4
        CUTOFF_Z = -0.5, 1
    
        data = np.frombuffer (msg.data, dtype=np.float32).reshape (-1, 4)
        data = data [ (data [:, 2] < CUTOFF_Z[1]) & (data [:, 2] > CUTOFF_Z[0]) ]
        
        xax, yax, zax = data[:, 0], data[:, 1], data[:, 2]
        
        radius = np.hypot (xax, yax)
        angle = np.arctan2 (yax, xax)
        angle = (np.degrees (angle).round().astype (np.int64, copy=False) + 360) % 360
        
        mdata = np.empty (360, np.float64)
        mdata.fill (np.nan)
        for i in range (0, 360):
        
            val = (angle == i)
            z_raw = zax    [val]
            r_raw = radius [val]
            #np.concatenate ([ z_raw, np.array (CUTOFF_Z) ])
            #np.concatenate ([ r_raw, np.array ([0          , np.inf]) ])
            
            if (z_raw.size < 2):
                continue
            
            idx = np.arange (0, z_raw.size)
            sortidx = np.argsort (z_raw, kind='mergesort')
            z_sort, r_sort = z_raw [sortidx], r_raw [sortidx]
            
            #z = np.linspace (CUTOFF_Z[0], CUTOFF_Z[1], 1000)
            #r = scipy.interpolate.spline (z_sort, r_sort, z)
            #r = self.moving_average (r_sort, r_sort.size / 100)
            #z = z_sort
            
            func = scipy.interpolate.interp1d (z_sort, r_sort, copy=False, assume_sorted=True, bounds_error=False, fill_value=np.inf)
            z = np.linspace (CUTOFF_Z[0], CUTOFF_Z[1], 500)
            r = func (z)
            r_ma = np.ma.array (r, mask=(np.isnan (r) | np.isinf (r)) )
            r = self.moving_average (r_ma, 5)
            
            with np.errstate (divide='ignore'):
                slope = 1 / np.gradient (r, z)
            
            #print r_ma, '\n', r, r.size, '\n', slope
            #assert False
            try:
                mdata[i] = r [slope > MAX_SLOPE][0]
            except IndexError:
                mdata[i] = np.inf
            
            '''
            print 'start'
            t = rospy.Time.now()
            
            func = scipy.interpolate.interp1d (y_raw, r_raw, copy=False, assume_sorted=True, bounds_error = False)
            y = np.arange (CUTOFF_Y[0], CUTOFF_Y[1], 0.001)
            r = func (y)
            
            t2 = rospy.Time.now()
            print (t2 - t).secs, (t2 - t).nsecs
            raw_input()
            '''
            '''
            idx = np.arange (0, y.size)
            sortidx = np.ma.argsort (y, kind='mergesort', fill_value=np.inf)
            y, r = y [sortidx], r [sortidx]
            '''
            '''
            y = np.arange (CUTOFF_Y[0], CUTOFF_Y[1], 0.001)
            
            t = rospy.Time.now()
            print rospy.Time.now() - t, 'ready to interp'
            t = rospy.Time.now()
            r = scipy.interpolate.spline (y_raw, r_raw, y)
            t2 = rospy.Time.now()
            print t2 - t, 'interp done'
            ''' '''
            print 'hello'
            fig = plt.figure()
            ax = fig.add_subplot (111)
            ax.scatter (r, z)
            ax.set_xlim (0, ax.get_xlim()[1])
            ax.set_ylim (-1, 1)
            fig.show()
            raw_input()
            
            fig = plt.figure()
            ax = fig.add_subplot (111)
            ax.scatter (r, slope)
            fig.show()
            raw_input()
            
            ''' '''
            #TODO, dy/dr, (also dr/di, where i is idx)
            slope = np.gradient (y, r)
            
            fig = plt.figure ()
            ax1, ax2, ax3, ax4 = fig.add_subplot (111), fig.add_subplot (111), fig.add_subplot (111), fig.add_subplot (111)
            #ax1.scatter (idx, r, color='orange')
            #ax2.scatter (idx, y, color='blue')
            ax3.scatter (r, y, color='green')
            #ax4.scatter (idx, slope, color='red')
            fig.show()
            print y.shape
            print np.max (r), np.max (y)
            raw_input()
            
            # note: sorting wrt y but slope wrt r
            # TODO find first invalid point
            # condition for invalid:
            # dr/di < 0 or dy/dr > MAX_SLOPE
            '''
            
        msg = navigation.msg.ObstacleMap ()
        msg.data = mdata
        self.pub.publish (msg)
        
        self.lock.release()
    
    def callback_flat (self, msg):
        
        data = np.frombuffer (msg.data, dtype=np.float32).reshape (-1, 4)
        
        line = data [np.logical_and (data[:, 2] > -0.005, data[:, 2] < 0.005), :]
        
        angle = np.arctan2 (line [:, 1], line[:, 0])
        angle [angle < 0] = 2*np.pi + angle [angle < 0]
        
        angle = np.degrees(angle).round().astype(np.int64, copy=False)
        
        dist = np.hypot (line [:, 0], line [:, 1])
        
        mdata = np.empty (360, np.float64)
        mdata.fill (np.nan)
        
        for i in range (360):
            try:
                mdata[i] = np.mean (dist [angle==i])
            except:
                mdata[i] = np.nan
        
        msg = navigation.msg.ObstacleMap()
        msg.data = mdata
        self.pub.publish (msg)
        
        '''
        # center only
        data = np.frombuffer (msg.data, dtype=np.float32).reshape (-1, 4)
        data = np.ma.array (data, mask = np.isnan (data))
        xax, yax, zax = data [:, 0], data [:, 1], data [:, 2]
        
        idx = np.argmin (np.hypot (yax, zax))
        print xax [idx], idx, yax[idx], zax[idx]
        '''
        
def run ():
    node = Node()
    rospy.spin()

run ()
