#!/usr/bin/env python
import rospy
import navigation.msg as navmsg
import navigation.srv as navsrv
import traversal.msg as travmsg
import threading

class Planner (object):

    def __init__ (self, freq=50):
        self.freq = freq
        self.exec_lock = threading.RLock()

        with self.exec_lock:
            self.target_dist = float ('nan')
            self.deviation = float ('nan')
            self.goal_dist = float ('nan')

            rospy.init_node ('planner')
            rospy.Subscriber ('target', navmsg.Target, self.target_cb)
            pub_drive = rospy.Publisher ('drive_inp', travmsg.WheelRpm, queue_size=10)
            pub_state = rospy.Publisher ('planner_state', navmsg.Planner_state, queue_size=1)
            rospy.Service ('Planner_state_ctrl', navsrv.plan_state, self.ctrl)

    def target_cb (self, msg):
        with self.exec_lock:
            for attr in ('target_dist', 'deviation', 'goal_dist'):
                setattr (self, attr, getattr(msg, attr))

    def ctrl (self, msg):
        raise NotImplementedError

    def spin (self):
        rate = rospy.Rate (self.freq)
        while not rospy.is_shutdown():
            with self.exec_lock:
                pass
                # calculate drive_inp
            rate.sleep()

if __name__ == '__main__':
    plan = Planner()
    plan.spin()
