#!/usr/bin/env python

HELP_STR = """
help not implemented yet

"""

"""
All indents are 4 spaces
"""

import sys
import rospy

import sensor_msgs.msg as sensor_msgs
import navigation.msg

import navigation.srv
import detection.srv

from gps_coords import GpsCoords

import threading
import signal
import os
import errno

from math import *

####################################################################################
####################################################################################

class GDMNode:

    """
    Subscribes to /fix, /gdm_command
    Publishes /goal
    Uses service Planner_state_ctrl
    Reads from terminal
    """
    
    def __init__(self, noplanner=False, nodetect=False, noarrow=False):
        
        # consts
        self.NODE_NAME = 'gdm_node'
        DIR_PATH = os.path.dirname(os.path.realpath(__file__))
        DIR_PATH = os.path.dirname(DIR_PATH) + '/config/'
        self.DEST_FILENAME = DIR_PATH + 'gps_data.txt'
        self.TELE_FILENAME = DIR_PATH + 'telemetry_coords.txt'
        
        # initialise state
        self.main = Main(publish_func = self.publish_goal,
                         planner_srv_call = self.planner_srv_call,
                         reached_srv_call = self.reached_srv_call,
                         arrow_srv_call = self.arrow_srv_call)
        rospy.init_node(self.NODE_NAME)
        
        # subscribe
        rospy.Subscriber('fix', sensor_msgs.NavSatFix, self.main.gps.update)
        rospy.Subscriber('planner_state', navigation.msg.Planner_state, self.main.planner_callback)
        
        # setup publisher
        self.goal_publisher = rospy.Publisher('goal', navigation.msg.Goal, queue_size=10)
        
        # setup service (unless specified disabled)
        if noplanner:
            rospy.loginfo ('GDM Node: Planner service will not be called')
            self.planner_service = None
        else:
            rospy.loginfo ('Waiting for planner service')
            rospy.wait_for_service('Planner_state_ctrl')
            self.planner_service = rospy.ServiceProxy('Planner_state_ctrl', navigation.srv.plan_state)
            rospy.loginfo ('Planner service acquired')
        
        if nodetect:
            rospy.loginfo ('GDM Node: Detection service will not be called')
            self.reached_service = None
        else:
            rospy.loginfo ('Waiting for detection service')
            rospy.wait_for_service('reached')
            self.reached_service = rospy.ServiceProxy('reached', detection.srv.reached)
            rospy.loginfo ('Detection service acquired')
        
        if noarrow:
            rospy.loginfo ('GDM Node: Arrow service not available')
            self.arrow_service = None
        else:
            rospy.loginfo ('Waiting for arrow service')
            rospy.wait_for_service('arrow')
            self.arrow_service = rospy.ServiceProxy('arrow', detection.srv.arrow)
            rospy.loginfo ('Arrow service acquired')

        # start wayfinding
        # wayfinding may be interrupted by terminal input or /gdm_command
        self.main.load(self.DEST_FILENAME)
        self.main.start()
        
        # any threads begun directly by GDMNode will use this to know when to exit
        self.is_shutdown = False
        
        # daemon thread which reads input from the terminal
        terminal_inp_thread = threading.Thread(target = self.terminal_inp_func)
        terminal_inp_thread.setDaemon(True)
        terminal_inp_thread.start()
        
        # for use in planner_srv_call
        self._psc_lastcont = False
        
        rospy.loginfo ('GDM Node Setup Complete')
        
    # end init
    ###############################
    
    
    # kills this object and all processes started by it
    def kill(self):
        rospy.loginfo ('GDM Node Exiting')
        self.main.kill()
        self.is_shutdown = True
        sys.exit(0)
    
    # checks if GDMNode is still running
    def is_alive(self):
        return not self.is_shutdown
    
    # handles commands published on /gdm_command
    def command_callback(self, msg):
        self.process_command(msg.data)
    
    # use this in a thread to handle terminal input
    # the thread should be marked as a daemon
    def terminal_inp_func(self):
    
        while not self.is_shutdown:
            inp = raw_input()
            self.process_command(inp)
    
    # parses a command string and carries out the desired operation
    def process_command(self, command):
    
        # available commands: start, pause, load, end, help
        # load  | without options loads coordinates from default file
        # load -t  | loads telemetry file
        # load fileaname  | loads given file
    
        words = command.split()
            
        if command == 'start':
            self.main.start()
        elif command == 'pause':
            self.main.pause()
        elif command == 'end':
            self.kill()
        
        elif command == 'load':
            self.main.load(self.DEST_FILENAME)
        
        elif len(words) == 2:
            if words[0] == 'load':
                if words[1] == '-t':
                    self.main.load(self.TELE_FILENAME)
                else:
                    self.main.load(words[1])
            elif words[0] == 'start' and words[1] == 'arrow':
                self.main.start (arrow=True)
        
        elif command == 'help':
            print HELP_STR
        
        # invalid command
        else:
            print 'available commands: start, pause, load, end, help'
        
    # publishes distance and bering to /goal
    def publish_goal(self, distance, bearing):
        goal = navigation.msg.Goal()
        goal.distance = distance
        goal.bearing = bearing
        self.goal_publisher.publish(goal)
    
    # calls the planner service
    def planner_srv_call(self, option):
        if not self.planner_service:
            return
        
        msg = navigation.srv.plan_stateRequest()
        if option == 'rst':
            msg.rst = 1
        elif option == 'contin':
            msg.contin = 1
        elif option == 'pause':
            msg.pause = 1
        else:
            return
        
        try:
            ret = self.planner_service(msg)
            
            if not msg.contin or (msg.contin and not self._psc_lastcont):
                rospy.loginfo ("Planner service called with option '"+option+"' returned "+ret.state)
            self._psc_lastcont = msg.contin
            
        except rospy.ServiceException:
            rospy.logerr ('Error calling planner service')
    
    # wrapper func for reached service
    def reached_srv_call (self):
        if not self.reached_service:
            return
        try:
            result = self.reached_service()
            rospy.loginfo ('Detection service complete')
        except rospy.ServiceException:
            rospy.logerr ('Error calling detection service /reached')
    
    def arrow_srv_call (self):
        if not self.arrow_service:
            rospy.logwarn ('Arrow service not available')
            return
        try:
            self.arrow_service()
            rospy.loginfo ('Arrow service complete')
        except rospy.ServiceException:
            rospy.logerr ('Error calling arrow service /arrow')
            

# end GDMNode
####################################################################################
####################################################################################

class Main:

    """
    Handles all the actual computing
    Has methods to start, pause and end as per requirement
    """

    # setup with the function used for publishing and to call the planner service
    # publish_func should take two args: distance, bearing
    def __init__(self, publish_func, planner_srv_call, reached_srv_call, arrow_srv_call):
        
        # output functions
        self.publish_func = publish_func
        self.planner_srv_call = planner_srv_call
        self.reached_srv_call = reached_srv_call
        self.arrow_srv_call = arrow_srv_call
        
        # current gps location
        self.gps = GpsCoords()
        
        # list of gps locations to read, set by self.load()
        # the current target is the element at index 0
        # elements are removed as the location is achieved
        self.dest_gps = []
        self.dest_gps_lock = threading.Lock()
        
        # variables which control the status of the main thread
        self.is_shutdown = False
        self.is_paused = True
        self.arrow = False  # arrow detect on this iteration?
        self.planner_status = 0  # current status of planner
        
        # main thread, will be initialised when self.start() is called for the first time
        self.main_thread = None

    # end init
    ###############################


    # (re)loads target gps coordinates from the given file
    def load(self, filename):
        print 'main loading from ' + filename
        
        # reset coords
        self.dest_gps_lock.acquire()
        self.dest_gps = []
        
        # read line by line from input file
        infile = None
        try:
            infile = open(filename, 'r')
            for line in infile.readlines():
            
                # ignore empty lines
                if line == '\n' or line == '':
                    continue
                
                lat, lon = [float(x) for x in line.split()]
                self.dest_gps.append(GpsCoords(lat, lon))
        
        # file not formatted correctly
        except ValueError:
            rospy.logerr ('Input file not formatted correctly')
            print 'Please check: ' + filename
            print 'Each line should have the form `latitude longitude`\n'
            self.dest_gps = []
        
        # possible invalid filename
        except IOError as e:
            # file not found
            if e.errno == errno.ENOENT:
                print e
                return
                
            # unexpected error
            else:
                raise
        
        # clean
        finally:
            self.dest_gps_lock.release()
            if infile:
                infile.close()
        
        rospy.loginfo ('Loading GPS locations')
        for point in self.dest_gps:
            print 'Loaded: '+str(point.latitude)+', '+str(point.longitude)
    
    # start calculating and publishing using given function
    # creates the main thread if it does not exist
    def start(self, arrow=False):
        self.arrow = arrow
        self.is_paused = False
        
        if not self.main_thread:
            self.main_thread = threading.Thread(target = self.run)
            self.main_thread.start()
        
        rospy.loginfo ('GDM Node: Main thread started')
        if (self.arrow):
            rospy.loginfo ('Will follow arrows after next GPS Coordinate')
    
    # stop publishing until start is called again
    def pause(self):
        self.is_paused = True
        rospy.loginfo ('GDM Node: Main thread paused')
    
    # end all threads begun by this class
    # once called, the same object cannot be reused
    def kill(self):
        rospy.loginfo ('GDM Node: Killing main thread')
        self.is_shutdown = True
    
    # updates the planner state
    def planner_callback(self, msg):
        self.planner_status = msg.status
        #print msg.status
    
    ###############################
    
    
    # main thread funciton
    def run(self):
        rate = rospy.Rate(2)

        # reset planner
        self.planner_srv_call('rst')
        self.planner_status = 0

        while not self.is_shutdown:
        
            rate.sleep()
            self.dest_gps_lock.acquire()
            
            # decide whether to run in this iteration
            if self.is_paused or len(self.dest_gps) == 0:
                pass
            
            # current goal reached?
            elif self.planner_status == 1:
                
                self.planner_srv_call('pause')
                self.planner_status = 0
                
                rospy.loginfo ('Reached GPS location '+str(self.dest_gps[0].latitude)+', '+str(self.dest_gps[0].longitude))
                
                rate.sleep()
                # run arrow service, if required
                if (self.arrow):
                    self.arrow_srv_call()
                    self.arrow = False
                
                # temporarily turn over control to the reached service to find the marker
                self.reached_srv_call()
                
                # update list of destinations
                self.dest_gps.pop(0)
                
                # reset planner
                self.planner_srv_call('rst')
                self.planner_status = 0
                
                # all loaded destinations completed?
                if len(self.dest_gps) == 0:
                    rospy.loginfo ('GDM Node: All destinations reached, pausing')
                else:
                    rospy.loginfo ('GDM Node: Next target: '+str(self.dest_gps[0].latitude)+', '+str(self.dest_gps[0].longitude))
                self.pause()
            
            # goal not yet reached
            else:
                # calculate and publish goal
                distance, bearing = self.calc(self.gps, self.dest_gps[0])
                self.publish_func(distance, bearing)
                self.planner_srv_call('contin')
            
            self.dest_gps_lock.release()
    
    # given src and dest GpsCoords, calculate target distance and bearing
    def calc(self, src, dest):
    
        lon1, lat1, lon2, lat2 = map(radians, [src.longitude, src.latitude, dest.longitude, dest.latitude])
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))
        dist_gps = 6371 * c*1000
        bearing = atan2(sin(lon2-lon1)*cos(lat2), (cos(lat1)*sin(lat2))-(sin(lat1)*cos(lat2)*cos(lon2-lon1)))
        bearing = degrees(bearing)
        
        return dist_gps,bearing

# end Main
####################################################################################
####################################################################################

# GDMNode object
node = None

# parse command line arguments and start the gdm node
def begin(argv):
    # parse argv
    argdict = {'noplanner':False, 'nodetect':False, 'help':False}
    for arg in argv:
        argdict[arg] = True
    
    # print help and exit
    if argdict['help']:
        print HELP_STR
        sys.exit(0)
    
    # no services should be called
    noplanner = False
    if argdict['noplanner']:
        noplanner = True
    
    nodetect = False
    if argdict['nodetect']:
        nodetect = True
    
    #DEBUG
    print argdict
    
    # start GDMNode
    global node
    node = GDMNode(noplanner, nodetect)
    
    # catch keyboard interrupts to shut down cleanly
    signal.signal(signal.SIGINT, end_program)
    
    # keep main thread alive to catch keyboard interrupts and cleanly exit
    while node.is_alive():
        pass
    
# cleanly ends the program
def end_program(signal, frame):
    global node
    node.kill()
    print
    sys.exit(0)

if __name__ == '__main__':
    begin(sys.argv[1:])













    
    

