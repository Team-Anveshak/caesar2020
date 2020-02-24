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

    # convienience methods
    begin_s =  staticmethod (lambda cmd: cmd.begin())
    pause_s =  staticmethod (lambda cmd: cmd.pause())
    resume_s = staticmethod (lambda cmd: cmd.resume())
    stop_s =   staticmethod (lambda cmd: cmd.stop())

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

        # for self.ctrl_server
        self._ctrl_server_argdict = {
            'start'  : self.start,
            'pause'  : self.pause,
            'resume' : self.resume,
            'stop'   : self.stop,
            'skip'   : self.skip
        }
        with self.exec_lock:
            spin_thread.start()
            rospy.Service ('gdm_ctrl', navsrv.GDMCtrl, self.ctrl_server)
            rospy.loginfo ('(init) GDM Node init complete')

    _cmd_ctrl_argdict = {
        'begin'  : (Command.begin_s, 'beginning [{}]'),
        'pause'  : (Command.pause_s, 'pausing'),
        'resume' : (Command.resume_s, 'resuming'),
        'stop'   : (Command.stop_s, 'stopping [{}]')
    }
    def cmd_ctrl (self, arg):
        '''
        Wrapper for control methods of the current command
        Silently andles exceptions and empty command list
        Raises KeyError if arg is not in GDMNode._ctrl_argdict
        '''
        with self.exec_lock:
            if self.killed or not len(self.cmds):
                return
            func, outstr = self._cmd_ctrl_argdict [arg]
            rospy.loginfo (('(cmd_ctrl) '+outstr).format(self.cmds[0]))
            try:
                func (self.cmds[0])
            except ControlFlowException as e:
                rospy.logerr ('(cmd_ctrl) {}'.format(e))

    def load (self, program):
        '''
        Loads the given program (list of strs)
        Raises StatementFormatException
        '''
        with self.exec_lock:
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

            self.cmd_ctrl ('stop')
            self.cmds_full = cmds
            rospy.loginfo ('(load) finished loading program')
            #DEBUG
            for cmd in self.cmds_full:
                print cmd

    def start (self):
        '''Starts or restarts loaded program'''
        with self.exec_lock:
            if self.killed:
                return
            if not len (self.cmds_full):
                rospy.logwarn ('(start) no program loaded yet')
                return
            self.cmd_ctrl ('stop')
            rospy.loginfo ('(start) (re)starting loaded program')
            self.cmds = list(self.cmds_full)
            # DEBUG:
            for cmd in self.cmds:
                print cmd
            self.cmd_ctrl ('begin')

    # TODO: better pause resume control
    def pause (self):
        '''Pauses the program'''
        with self.exec_lock:
            self.cmd_ctrl ('pause')

    def resume (self):
        '''Resumes paused program'''
        with self.exec_lock:
            self.cmd_ctrl ('resume')

    def stop (self):
        '''Stops the current program and idles the node'''
        with self.exec_lock:
            if self.killed:
                return
            if not len (self.cmds):
                rospy.logwarn ('(stop) program not running')
                return
            self.cmd_ctrl ('stop')
            rospy.loginfo ('(stop) stopping program')
            self.cmds = []

    # TODO: skip should retain paused state
    def skip (self):
        '''Skips the current command'''
        with self.exec_lock:
            if self.killed:
                return
            if not len (self.cmds):
                rospy.logwarn ('(skip) program not running')
                return
            self.cmd_ctrl ('stop')
            self.cmds.pop(0)
            if not len (self.cmds):
                rospy.loginfo ('program complete')
            else:
                self.cmd_ctrl ('begin')

    def kill (self):
        '''Signals the end of the spin thread'''
        with self.exec_lock:
            self.cmd_ctrl ('stop')
            rospy.loginfo ('(kill) killing main thread')
            self.killed = True

    def ctrl_server (self, arg):
        '''Server for gdm_ctrl service'''
        # input can be service message or raw str
        arg = getattr (arg, 'data', arg)
        with self.exec_lock:
            try:
                func = self._ctrl_server_argdict[arg]
            except KeyError:
                rospy.logerr ('(ctrl_server) invalid arg [{}]'.format(arg))
                return False
            func()
            return True

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
                    rospy.loginfo ('(spin) `{}` finished'.format(cmd))
                    if len (self.cmds):
                        self.cmd_ctrl ('begin')
                    else:
                        rospy.loginfo ('(spin) program complete')
            rate.sleep()
