#!/usr/bin/env python

HELP_STR = """
ROS Node which logs the current GPS location on command
Coords are logged to gps_coords.txt
Saves the log file in the same directory as the logger.py file

Command line args
help: Print this help and exit
append: Append to existing log file instead of overwriting it

Press 'Enter' while the program is running to manually log present coordinates

Subscriptions:
/fix of type sensor_msgs/NavSatFix (to obtain GPS coordinates)
/log_gps_now of type std_msgs/Empty (command to log present coords)

Publications:
/telemetry_status of type std_msgs/int16 (publish 1 during logging at the rate of 1 Hz, 0 when it ends)

Requires gps_coords.py to be in the same directory

"""

"""
All indents are 4 spaces
"""

import rospy
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import gps_coords
from threading import Thread

# for command line args, exit
import sys

# to determine where to save the log file
import os

# to catch keyboard interrupts
import signal

# the name of this ROS node
NODE_NAME = 'gps_logger'

# log file path: "[location of python file]/LOG_FILENAME"
LOG_FILENAME = 'telemetry_coords.txt'
LOG_DIR = os.path.dirname(os.path.realpath(__file__))
LOG_DIR = os.path.dirname(LOG_DIR) + '/config/'
LOG_FILE_PATH = LOG_DIR + LOG_FILENAME


# globals, to set later
file_mode = ''  # log file is opened in this mode
status_pub = None   # publishes start and end of logging
shutdown = False    # threads use this to know when to stop

# object to keep track of current GPS location
gps_fix = gps_coords.GpsCoords()

# opens the log file as given by file_mode
# writes present coordinates and closes the file
def log_now_call_back(msg):

    global file_mode
    
    log_file = None
    try:
        log_file = open(LOG_FILE_PATH, file_mode)
        new_entry = str(gps_fix.latitude) + ' ' + str(gps_fix.longitude)
        log_file.write(new_entry + '\n')
    finally:
        if log_file:
            log_file.close()
    
    print NODE_NAME + ': wrote ' + new_entry
    
    # after running once, all future changes should be appended
    file_mode = 'a'

# continuously publishes 1 to /telemetry_status, indicating telemetry is going on
# use this in a separate thread
# publishes 0 and quits when global shutdown is set
def publish_telemetry_status(publisher):
    global shutdown
    
    rate = rospy.Rate(1)
    while not shutdown:
        publisher.publish(1)
        rate.sleep()
    
    publisher.publish(0)

# sets up subscriptions and callbacks
def setup_logger():
    rospy.init_node(NODE_NAME)
    
    rospy.Subscriber('fix', sensor_msgs.NavSatFix, gps_fix.update)
    rospy.Subscriber('log_gps_now', std_msgs.Empty, log_now_call_back)
    
    global status_pub
    status_pub = rospy.Publisher('telemetry_status', std_msgs.Int16, queue_size=10)
    signal.signal(signal.SIGINT, end_program)
    
    # keep indicating that telemetry is in progress
    status_pub_thread = Thread(target = publish_telemetry_status, args = [status_pub])
    status_pub_thread.start()
    
    # also accept gps-log commands from stdin
    while True:
        raw_input('--- Press Enter to log present coordinates ---')
        log_now_call_back(None)

# publishes end of telemetry and quit
def end_program(signal, frame):
    global shutdown
    shutdown = True
    print
    sys.exit(0)

if __name__ == '__main__':
    
    # parse command line options
    options = {'append': False, 'help': False}
    for arg in sys.argv[1:]:
        if arg == 'append':
            options['append'] = True
        elif arg == 'help':
            options['help'] = True
    
    # print help and exit
    if options['help']:
        print HELP_STR
        sys.exit(0)
    
    # do not overwrite existing log file
    if options['append']:
        file_mode = 'a'
    else:
        file_mode = 'w'
    
    print 'Writing to: ' + LOG_FILE_PATH
    if file_mode == 'w':
        print 'Overwriting existing contents'
    else:
        print 'Appending to existing contents'
    
    setup_logger()



