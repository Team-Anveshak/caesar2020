#!/usr/bin/env python

HELP_STR = """
ROS GDM Node for Autonomous Task

Subscriptions:
/fix of type sensor_msgs/NavSatFix (to obtain GPS coordinates)
/planner_state of type navigation/Planner_state (to determine waypoint completion)
/telemetry_status of type std_msgs/Int16 (to determine if telemetry is running)

Publications:
/goal of type navigation/Goal (bearing and distance relative to next waypoint)

REPLACEMENT FOR THE gdm_node.py file
------------------------------------------
THIS CODE HAS NOT BEEN TESTED ON THE ROVER
------------------------------------------

"""

"""
All indents are 4 spaces
"""

import rospy
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import navigation.msg
from gps_coords import GpsCoords

import navigation.srv

import os
from threading import Thread
import sys
import signal

# used in the waypoints.calc function
from math import *

NODE_NAME = 'gdm'


# keeps track of all waypoints, publishes to /goal
class Waypoints:
    def __init__(self):
    
        # telmetry file used to store waypoints obtained through telemtry
        # dest file used to obtain user-specified (URC-specified) waypoints
        # both files are stored in the same directory as this python file
        
        self.DIR_PATH = os.path.dirname(os.path.realpath(__file__)) + '/'
        self.TELEMETRY_FILENAME = self.DIR_PATH + 'telemetry_coords.txt'
        self.DEST_FILENAME = self.DIR_PATH + 'dest_coords.txt'
    
        # list of GpsCoords()
        self.points = []
        self.curr_point = 0     # index of current point
        
        # state variables, volatile
        self.gps = GpsCoords()  # current gps location
        self.planner = 0        # has current waypoint been reached? (0:false)

        # publisher for /goal
        self.goal_pub = None
        
        # this variable determines if wayfinding is in progress
        # changing this to False will interrupt wayfinding
        self.wayfinding = False
        
        # setting this to True will force self.begin() to exit
        self.stopped = False
        
        # indicates whether the file being read from is the telemetry file
        self.telemetry_follow = False
    
    def planner_state_callback(self, planner_state):
        self.planner = planner_state.status
    
    # (re)loads the list of waypoints
    # if telemetry is True, loads points from the telemetry file
    def load(self, telemetry=False):
        
        # reset coords
        self.points = []
        
        # decide which file to open
        filename = self.DEST_FILENAME
        self.telemetry_follow = False
        if telemetry:
            filename = self.TELEMETRY_FILENAME
            self.telemetry_follow = True
        
        # read line by line from input file
        coord_file = None
        try:
            coord_file = open(filename, 'r')
            for line in coord_file.readlines():
            
                # ignore empty lines
                if line == '\n' or line == '':
                    continue
                
                lat, lon = [float(x) for x in line.split()]
                self.points.append(GpsCoords(lat, lon))
        
        # file not formatted correctly
        except ValueError:
            print 'Input file not formatted correctly'
            print 'Please check: ' + filename
            print 'Each line should have the form `latitude longitude`\n'
            raise
        
        # clean
        finally:
            if coord_file:
                coord_file.close()
        
        # reset current index
        self.curr_point = 0
        
        for point in self.points:
            print point.latitude, point.longitude
    
    # begin wayfinding
    # this function is typically running throughout the life of the program
    def begin(self):
    
        self.wayfinding = True
        
        # planner_stopped indicates if 'contin' needs to be run on the planner
        self.call_planner_srv('rst')
        planner_stopped = True
        
        rate = rospy.Rate(1)
        while not self.stopped:
            
            rate.sleep()
            
            if self.wayfinding:
                
                # current goal reached?
                if self.planner == 1:
                    print 'Reached GPS location',
                    print self.points[self.curr_point].latitude, self.points[self.curr_point].longitude
                    
                    self.curr_point += 1
                    self.call_planner_srv('rst')
                    planner_stopped = True
                    self.planner = 0
                    
                    # all loaded points completed?
                    if self.curr_point >= len(self.points):
                        print 'All coordinates reached'
                        
                        # if reading from telemetry file, go back to original file
                        if self.telemetry_follow:
                            self.load()
                        
                        # unload all points and pause the node
                        else:
                            self.points = []
                            self.curr_point = 0
                            self.pause()
                        
                        # don't proceed with goal publishing in this iteration
                        continue
                
                # calculate and publish to /goal
                goal = navigation.msg.Goal()
                goal.distance, goal.bearing = self.calculate(self.gps, self.points[self.curr_point])
                self.goal_pub.publish(goal)
                
                # check if planner needs to be restarted
                if planner_stopped:
                    self.call_planner_srv('contin')
                    planner_stopped = False
            
            # wayfinding is currently paused, don't publish or run the planner
            else:
                self.call_planner_srv('pause')
                planner_stopped = True
    
    # given source and destination GpsCoords, calculates distance and bearing
    # extensive use of math module
    def calculate(self, src, dest):
    
        lon1, lat1, lon2, lat2 = map(radians, [src.longitude, src.latitude, dest.longitude, dest.latitude])
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))
        dist_gps = 6371 * c*1000
        bearing = atan2(sin(lon2-lon1)*cos(lat2), (cos(lat1)*sin(lat2))-(sin(lat1)*cos(lat2)*cos(lon2-lon1)))
        bearing = degrees(bearing)
        
        return dist_gps,bearing
    
    # options can be one of 'rst', 'contin', 'pause'
    def call_planner_srv(self, option):
        
        global noservice
        if noservice:
            return
        
        # prepare message to send
        msg = navigation.msg.Planner_stateRequest()
        
        if option == 'rst':
            print 'Resetting planner'
            msg.rst = 1
            
        elif option == 'contin':
            print 'Continuing planner'
            msg.rst = 1
        
        elif option == 'pause':
            print 'Pausing planner'
            msg.pause = 1
        
        # invalid option, do nothing
        else:
            return
        
        try:
            resp = planner_srv(msg)
            return resp
        except rospy.ServiceException:
            print 'Unable to call planner service'
            return 'Error'
        
    
    # pause operations, for example when obtaining waypoints through telemetry
    def pause(self):
        self.wayfinding = False
    
    # resume operations after a call to self.pause()
    def resume(self):
        self.wayfinding = True
        
    # stop operations. do this to cleanly end the program
    def stop(self):
        self.stopped = True
    
    # returns True if wayfinding is in progress, returns False if paused or not begun
    def is_running(self):
        return self.wayfinding



# globals
waypints = None
planner_srv = None

# this option can be set from the command line arguments
# if set, no services will be called
noservice = False

# wrapper for state variables (volatile)
# variables may be altered by subscribed messages

# interrupts wayfinding during telemetry
# resumes wayfinding when telemetry is over
def telemetry_state_callback(msg):
    global waypoints
    
    if (msg.data == 1):
        if waypoints.is_running():
            waypoints.pause()
            print 'Wayfinding interrupted, telemetry in progress'
    
    # receiving zero will reload telemetry waypoints even if wayfinding was not paused
    elif (msg.data == 0):
        if waypoints.is_running():
            waypoints.pause()
            print 'Wayfinding interrupted, preparing to load telemetry coordinates'
        
        waypoints.load(telemetry=True)
        print 'Wayfinding resumed with telemetry coordinates'
        waypoints.resume()
    

# initialise the gdm node
def setup_gdm():
    rospy.init_node(NODE_NAME)
    global waypoints
    
    # handled by the Waypoints class
    rospy.Subscriber('fix', sensor_msgs.NavSatFix, waypoints.gps.update)
    rospy.Subscriber('planner_state', navigation.msg.Planner_state, waypoints.planner_state_callback)
    
    # handled outside the Waypoints class
    rospy.Subscriber('telemetry_status', std_msgs.Int16, telemetry_state_callback)
    
    waypoints.goal_pub = rospy.Publisher('goal', navigation.msg.Goal, queue_size=10)
    
    # setup planner service
    if not noservice:
        rospy.wait_for_service('Planner_state_ctrl')
        global planner_srv
        planner_srv = rospy.ServiceProxy('Planner_state_ctrl', navigation.srv.plan_state)


def end_program(signal, frame):
    global waypoints
    waypoints.stop()
    sys.exit(0)


if __name__ == '__main__':

    # parse command line options
    options = {'noservice': False, 'help': False}
    for arg in sys.argv[1:]:
        if arg == 'noservice':
            options['noservice'] = True
        elif arg == 'help':
            options['help'] = True
    
    # print help and exit
    if options['help']:
        print HELP_STR
        sys.exit(0)
    
    # do not run any services (specifically, planner service)
    if options['noservice']:
        print 'noservice: No services will be called'
        noservice = True

    global waypoints
    waypoints = Waypoints()

    # catch keyboard interrupts
    signal.signal(signal.SIGINT, end_program)

    # init
    setup_gdm()
    
    # load coordinates and begin pathfinding
    waypoints.load()
    print 'Wayfinding begun with user-specified coordinates'
    waypoints.begin()
    
    
    
    
    
    
    
    
    
    
