#!/usr/bin/env python

import rospy
import threading, signal, sys
from gdm_server import GDMNode, StatementFormatException

progstr = '''
sleep 1000
goal 45 21
sleep 3000
nav 120 90
'''

class Frontend:
    def __init__ (self):
        self.node = GDMNode()
        signal.signal (signal.SIGINT, self.kill)

        global progstr
        self.node.load (progstr.split('\n'))

        self.spin()

    def kill (self, signal=None, frame=None):
        self.node.kill()
        sys.exit(0)

    def spin (self):
        while not rospy.is_shutdown():
            try:
                inp = raw_input().strip()
            except EOFError:
                self.kill()
            if inp in ['start', 'pause', 'stop', 'pause', 'resume', 'skip']:
                self.node.ctrl_server(inp)
            elif inp == 'load':
                inprog = []
                while not rospy.is_shutdown():
                    try:
                        inprog.append (raw_input().strip())
                    except EOFError:
                        break
                try:
                    self.node.load (inprog)
                except StatementFormatException:
                    pass

            elif len (inp):
                try:
                    self.node.load ([inp])
                except StatementFormatException:
                    pass
        print 'end of the line'


Frontend()
