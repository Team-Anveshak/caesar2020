#!/usr/bin/env python

import rospy
import threading, signal, sys
from _backend_gdm_node import GDMNode

class Frontend:
    def __init__ (self):
        self.node = GDMNode()
        signal.signal (signal.SIGINT, self.kill)
        self.node.load ([
            'sleep 1000',
            'goal 45 21',
            'sleep 3000',
            'nav 120 90',
        ])
        self.node.begin()

        self.spin()

    def kill (self, signal=None, frame=None):
        self.node.kill()
        sys.exit(0)

    def spin (self):
        while True:
            inp = raw_input().strip()
            if inp == 'pause':
                self.node.pause()
            elif inp == 'resume':
                self.node.resume()
            elif inp == 'stop':
                self.node.stop()
            elif inp == 'reload':
                self.node.reload()
            elif inp == 'kill':
                self.kill()
            elif inp == 'begin':
                self.node.begin()

Frontend()
