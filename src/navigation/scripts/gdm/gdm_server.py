import rospy
import threading
import navigation.msg as navmsg
import navigation.srv as navsrv
import numpy as np

class StatementFormatException (Exception):
    '''Program statements passed to GDMNode are formatted incorrectly'''
    pass

class ControlFlowException (Exception):
    '''Error in changing the current command state'''
    pass

class ROSNode (object):
    '''Wraps the interface between GDM and other autonomous nodes'''

    # short namespace for msg constants
    _avoid = navsrv.AvoidNodeCtrlRequest

    def __init__ (self):
        rospy.init_node ('gdm_node')

        self.planner_reached = False
        rospy.Subscriber ('planner_state', navmsg.Planner_state, self.planner_cb)

        self.plan_srv = rospy.ServiceProxy ('Planner_state_ctrl', navsrv.plan_state)
        self.avoid_srv = rospy.ServiceProxy ('avoid_node_ctrl', navsrv.AvoidNodeCtrl)

    def planner_cb (self, msg):
        self.planner_reached = (msg.status == 1)


class Command (object):
    '''
    Each command type is represented as an instance of a Command subclass
    Note that Command and subclasses are not thread safe

    Subclasses should implement:
    class variables:
        CMD_STRINGS: list of all strings recognised as this command
    methods:
        begin: reset everytime this function is called
        pause: may raise ControlFlowException if called before begin
        resume: may raise ControlFlowException if called before begin
        stop: reset to state before begin is called for the first time

        spinOnce: this function will be called once every loop iteration
            returns bool: True if command was completed on this iteration
            False if not begun or or not completed
            On returning True, reset to state before begin.

        __str__: should be overridden
    '''

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
    '''Do nothing for the specified number of milliseconds'''

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
        if self.tstart is None:
            raise ControlFlowException ('[{}] tried to pause before begin'.format(self))
        self.t = rospy.Time.now().to_nsec() / self.ONEM
        self.tvar += t - self.tlast
        self.tlast = None

    def resume (self):
        if self.tstart is None:
            raise ControlFlowException ('[{}] tried to resume before begin'.format(self))
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
    '''
    Navigate to the gps point (latitude, longitude)
    If command is 'nav' perform obstacle avoidance
    If command is 'goal' assume no obstacles
    '''

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
        try:
            self.resume()
        finally:
            self.node.planner_reached = False

    def pause (self):
        try:
            self.node.plan_srv (pause=0, contin=0, rst=1)
            self.node.avoid_srv (ROSNode._avoid.PAUSE, self.lat, self.lon)
        except rospy.ServiceException as e:
            raise ControlFlowException(e)

    def resume (self):
        try:
            self.node.avoid_srv (self.def_mode, self.lat, self.lon)
            self.node.plan_srv (pause=0, contin=0, rst=1)
            self.node.plan_srv (pause=0, contin=1, rst=0)
        except rospy.ServiceException as e:
            raise ControlFlowException(e)

    def stop (self):
        self.pause()

    # returns True when command is complete
    def spinOnce (self):
        if self.node.planner_reached:
            try:
                self.stop()
            finally:
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
            rospy.Service ('gdm_ctrl', navsrv.GDMCtrl, self.ctrl)
            rospy.loginfo ('GDM Node init complete')

    def load (self, program):
        '''
        Loads the given program (list of strs)
        Raises StatementFormatException
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

    ##############################
    # called by self.ctrl (DO NOT CALL _* functions directly)

    def _begin (self, cmd):
        cmd.begin(cmd)

    def _pause (self, cmd):
        cmd.pause(cmd)

    def _resume (self, cmd):
        cmd.resume(cmd)

    def _stop (self, cmd):
        cmd.stop(cmd)
        self.cmds = []

    def _skip (self, cmd):
        #TODO
        cmd.stop(cmd)
        self.cmds.pop(0)
    ##############################

    # arg may be str or navsrv.GDMCtrl
    _ctrl_argdict = {
        'begin':  (self._begin,  '[{}] started'),
        'pause':  (self._pause,  'pausing'),
        'resume': (self._resume, 'resuming'),
        'stop':   (self._stop,   'program stopped')
    }
    def ctrl (self, arg):
        try:
            if not isinstance (arg, str):
                arg = arg.data
            func, outstr = self._ctrl_argdict[arg]
        except (KeyError, AttributeError):
            rospy.logerr ('invalid arg [{}] to gdm node ctrl'.format(arg))
            return False

        with self.exec_lock:
            if len (self.cmds) and not self.killed:
                rospy.loginfo (outstr.format(self.cmds[0]))
                try:
                    func(self.cmds[0])
                except ControlFlowException as e:
                    rospy.logerr (e)
                    return False

        return True

    def kill (self):
        with self.exec_lock:
            self.ctrl ('stop')
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
                        self.cmd[0].begin()
                    else:
                        rospy.loginfo ('program complete')
            rate.sleep()
