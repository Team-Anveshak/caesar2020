import rospy
import threading
import navigation.msg
import navigation.srv
import numpy as np

class StatementFormatException (Exception):
    pass

class ROSNode (object):
    # short namespace for msg constants
    _avoid = navigation.srv.AvoidNodeCtrlRequest

    def __init__ (self):
        rospy.init_node ('gdm_node')

        self.planner_reached = False
        rospy.Subscriber ('planner_state', navigation.msg.Planner_state, self.planner_cb)

        self.plan_srv = rospy.ServiceProxy ('Planner_state_ctrl', navigation.srv.plan_state)
        self.avoid_srv = rospy.ServiceProxy ('avoid_node_ctrl', navigation.srv.AvoidNodeCtrl)

    def planner_cb (self, msg):
        self.planner_reached = (msg.status == 1)

# superclass for all commands
# note that Command and subclasses are not thread safe
class Command (object):

    @staticmethod
    def get_instance (node, stmt):
        '''
        Returns a Command subclass representing the program statement `stmt`
        Throws StatementFormatException
        '''
        tokens = stmt.split()
        if not len(tokens):
            return None

        for Cls in Command.__subclasses__():
            if tokens[0] in Cls.CMD_STRINGS:
                try:
                    return Cls (node, tokens)
                except (ValueError, AssertionError):
                    raise StatementFormatException ('invalid arguments to `{}`'.format(tokens[0]))

        raise StatementFormatException ('invalid command type `{}`'.format(tokens[0]))

class Sleep (Command):
    CMD_STRINGS = ['sleep']
    ONEM = 1000000

    # sleep msecs
    def __init__ (self, node, argv):
        assert len (argv) == 2
        assert argv[0] in self.CMD_STRINGS
        self.msecs = int (argv[1])
        self.tstart = None
        self.tvar = 0
        self.tlast = None
        self._str = ' '.join(argv)

    def begin (self):
        self.tstart = rospy.Time.now().to_nsec() / self.ONEM
        self.tlast = self.tstart
        self.tvar = 0

    def pause (self):
        self.t = rospy.Time.now().to_nsec() / self.ONEM
        self.tvar += t - self.tlast
        self.tlast = None

    def resume (self):
        self.tlast = rospy.Time.now().to_nsec() / self.ONEM
        self.running = True

    def stop (self):
        self.tstart = None
        self.tvar = 0
        self.tlast = None

    # returns True once the command has finished
    def spinOnce (self):
        if (not self.tstart) or (not self.tlast):
            return False

        t = rospy.Time.now().to_nsec() / self.ONEM
        self.tvar += t - self.tlast
        self.tlast = t

        if self.tvar >= self.msecs:
            self.stop()
            return True
        return False

    def __str__(self):
        return self._str

class Navigate (Command):
    CMD_STRINGS = ['nav', 'goal']

    # goal lat lon
    # nav lat lon
    def __init__ (self, node, argv):
        assert len (argv) == 3
        assert argv[0] in self.CMD_STRINGS
        self.gps = self.lat, self.lon = float (argv[1]), float (argv[2])

        self.node = node
        if argv[0] == 'nav':
            self.def_mode = ROSNode._avoid.AVOID_OBSTACLES
        elif argv[0] == 'goal':
            self.def_mode = ROSNode._avoid.IGNORE_OBSTACLES

        self._str = ' '.join (argv)

        self.running = False

    def begin (self):
        self.resume()
        self.node.planner_reached = False

    def pause (self):
        try:
            self.node.plan_srv (pause=0, contin=0, rst=1)
            self.node.avoid_srv (ROSNode._avoid.PAUSE, self.lat, self.lon)
        except rospy.ServiceException as e:
            rospy.logerr ('service error: '+str(e))

    def resume (self):
        try:
            self.node.avoid_srv (self.def_mode, self.lat, self.lon)
            self.node.plan_srv (pause=0, contin=0, rst=1)
            self.node.plan_srv (pause=0, contin=1, rst=0)
        except rospy.ServiceException as e:
            rospy.logerr ('service error: '+str(e))

    def stop (self):
        self.pause()

    # returns True when command is complete
    def spinOnce (self):
        if self.node.planner_reached:
            self.stop()
            return True
        return False

    def __str__ (self):
        return self._str

class GDMNode (object):

    def __init__ (self, freq=10):
        self.node = ROSNode()

        self.FREQ = freq
        self.exec_lock = threading.RLock()
        self.cmds_full = []
        self.cmds = []

        self.killed = False
        spin_thread = threading.Thread (target = self.spin)
        with self.exec_lock:
            spin_thread.start()
            rospy.loginfo ('GDM Node init complete')

    def load (self, program):
        '''
        Loads the given program (list of strs)
        Throws StatementFormatException
        '''
        if self.killed:
            return

        cmds = []
        for i, stmt in enumerate (program):
            try:
                cmd = Command.get_instance (self.node, stmt)
            except StatementFormatException as e:
                raise StatementFormatException ('error in line {}: {}'.format(i+1, e))
            if cmd:
                cmds.append (cmd)

        with self.exec_lock:
            self.stop()
            self.cmds_full = cmds
            self.cmds = list (self.cmds_full)
            rospy.loginfo ('finished loading program')
            #DEBUG
            for cmd in self.cmds_full:
                print cmd

    def reload (self):
        with self.exec_lock:
            if self.killed:
                return
            self.stop()
            rospy.loginfo ('loading last program')
            self.cmds = list (self.cmds_full)

    def begin (self):
        with self.exec_lock:
            if len (self.cmds) and not self.killed:
                rospy.loginfo ('`{}` started'.format(self.cmds[0]))
                self.cmds[0].begin()

    def pause (self):
        with self.exec_lock:
            if len (self.cmds) and not self.killed:
                rospy.loginfo ('pausing')
                self.cdms[0].pause()

    def resume (self):
        with self.exec_lock:
            if len (self.cmds) and not self.killed:
                rospy.loginfo ('resuming')
                self.cmds[0].resume()

    def stop (self):
        with self.exec_lock:
            if len (self.cmds) and not self.killed:
                rospy.loginfo ('program stopped')
                self.cmds[0].stop()
                self.cmds = []

    def kill (self):
        with self.exec_lock:
            self.stop()
            rospy.loginfo ('killing main thread')
            self.killed = True

    def spin (self):
        rate = rospy.Rate (self.FREQ)
        while True:
            with self.exec_lock:
                if self.killed:
                    break
                if not len (self.cmds):
                    continue
                complete = self.cmds[0].spinOnce()
                if complete:
                    cmd = self.cmds.pop(0)
                    rospy.loginfo ('`{}` finished'.format(cmd))
                    if len (self.cmds):
                        self.begin()
                    else:
                        rospy.loginfo ('program complete')
            rate.sleep()
