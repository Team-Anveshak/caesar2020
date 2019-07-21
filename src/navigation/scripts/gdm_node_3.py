#!/usr/bin/env python

"""
--- IMPLEMENTATION IS NOT COMPLETE ---
"""

HELP_STR = """
help not implemented yet

"""

"""
All indents are 4 spaces
"""

import sys
import rospy

import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import navigation.msg

from gps_coords import GpsCoords

from threading import Thread
import signal

###################################################################

class GDMNode:

    """
    Subscribes to /fix, /gdm_command
    Publishes /goal
    Uses service Planner_state_ctrl
    Reads from terminal
    """
    
    def __init__(self, noservice = False):
        
        # consts
        self.NODE_NAME = 'gdm_node'
        self.DEST_FILENAME = '' #TODO
        self.TELE_FILENAME = '' #TODO
        
        # initialise state
        self.main = Main(publish_func = self.publish_goal)
        rospy.init_node(self.NODE_NAME)
        
        # subscribe
        rospy.Subscriber('fix', sensor_msgs.NavSatFix, self.main.gps.update)
        rospy.Subscriber('gdm_command', std_msgs.String, self.command_callback)
        
        # setup publisher
        self.goal_publisher = rospy.Publisher('goal', navigation.msg.Goal, queue_size=10)
        
        # start wayfinding
        # wayfinding may be interrupted by terminal input or /gdm_command
        self.main.load(self.DEST_FILENAME)
        self.main.start()
        
        # any threads begun directly by GDMNode will use this to know when to exit
        self.is_shutdown = False
        
        # thread which reads input from the terminal
        terminal_inp_thread = Thread(target = self.terminal_inp_func)
        terminal_inp_thread.start()
        
    # use this in a thread to handle terminal input
    def terminal_inp_func(self):
        #TODO
        while not self.is_shutdown:
            pass
    
    def planner_command(self, option):
        #TODO
        pass
    
    # kills this object and all processes started by it
    def kill(self):
        self.main.kill()
        self.is_shutdown = True
    
    # handles commands published on /gdm_command
    def command_callback(self, msg):
        #TODO
        print msg.data
        
    # publishes distance and bering to /goal
    def publish_goal(self, distance, bearing):
        goal = navigation.msg.Goal()
        goal.distance = distance
        goal.bearing = bearing
        self.goal_publisher.publish(goal)

# end GDMNode
###################################################################

class Main:

    """
    Handles all the actual computing
    Has methods to start, pause and end as per requirement
    """

    # setup with the function used for publishing
    # publish_func should take two args: distance, bearing
    def __init__(self, publish_func):
        #TODO
        
        self.publish_func = publish_func
        
        # current gps location
        self.gps = GpsCoords()

    # (re)loads target gps coordinates from the given file
    def load(self, filename):
        #TODO
        pass
    
    # start calculating and publishing using given function
    def start(self):
        #TODO
        pass
    
    # stop publishing until start is called again
    def pause(self):
        #TODO
        pass
    
    # end all threads begun by this class
    def kill(self):
        #TODO
        pass
    
    # main thread funciton
    def run(self):
        #TODO
        pass
    
    # given src and dest GpsCoords, calculate target distance and bearing
    def calc(self, src, dest):
        #TODO
        pass

# end Main
###################################################################

# GDMNode object
node = None

# parse command line arguments and start the gdm node
def begin(argv):
    # parse argv
    argdict = {'noservice':False, 'help':False}
    for arg in argv:
        argdict[arg] = True
    
    # print help and exit
    if argdict['help']:
        print HELP_STR
        sys.exit(0)
    
    # no services should be called
    noservice = False
    if argdict['noservice']:
        print 'Services will not be called'
        noservice = True
    
    #DEBUG
    print argdict
    
    # start GDMNode
    global node
    node = GDMNode(noservice)
    
    # catch keyboard interrupts to shut down cleanly
    signal.signal(signal.SIGINT, end_program)
    
    # keep main thread alive to catch keyboard interrupts and cleanly exit
    rospy.spin()
    
# cleanly ends the program
def end_program(signal, frame):
    global node
    node.kill()
    print
    sys.exit(0)

if __name__ == '__main__':
    begin(sys.argv[1:])













    
    
