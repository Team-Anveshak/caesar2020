#!/usr/bin/env python

import rospy
from _backend_gdm_node import GDMNode

def run ():
    node = GDMNode()
    node.load ([
        'sleep 1000',
        'goal 45 21',
        'sleep 3000',
        'nav 120 90',
    ])
    node.begin()
    while not rospy.is_shutdown():
        pass
    node.kill()

run()
