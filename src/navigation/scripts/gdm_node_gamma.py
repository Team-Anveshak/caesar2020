#!/usr/bin/env python

import rospy
import navigation.srv
import navigation.msg

import threading

FREQ = 10

class Statement:
    def __init__ (self, cmd, **kwargs):
        self.__dict__ = kwargs
        self.cmd = cmd

class ProgramFormatException (Exception):
    pass

class Manager:
    def __init__ (self, node):
        self.node = node
    
    def run (self):
        commands = [
            'goal 32.4 12.1',
            'sleep 1000',
            'nav -123 0',
            'sleep 2000']
        self.node.load (commands)
        self.node.begin()
        while not rospy.is_shutdown():
            pass
        self.node.kill()

class GDMNode:
    def __init__(self, freq):
        # main thread state variables
        self.NEXT = 0        # exec new stmt in next iteration
        self.NEWCMD = 1      # exec new stmt in this iteration
        self.PAUSE = 2       # pause exec in this iteration, goto sleeping
        self.SLEEPING = 3    # do nothing
        self.RESUME = 4      # recover from sleep
        self.RUNNING = 5     # stmt exec in progress
        self.KILL = 6        # clean up in next iteration, then quit
        
        self.freq = freq
        
        self.program = []
        self.state = self.SLEEPING
        self.stmt = Statement ('')
        self.lock = threading.Lock()
        
        self.sync_lock = threading.Lock()
        
        rospy.init_node ('gdm_node')
        rospy.Subscriber ('planner_state', navigation.msg.Planner_state, self.planner_cb)
        self.plan_state = 0
        
        self.plan_srv = rospy.ServiceProxy ('Planner_state_ctrl', navigation.srv.plan_state)
        self.avoid_srv = rospy.ServiceProxy ('avoid_node_ctrl', navigation.srv.AvoidNodeCtrl)
        
        thread = threading.Thread (target=self.run)
        thread.start()
        
        rospy.loginfo ('GDM Node init complete')
    
    def planner_cb (self, msg):
        self.plan_state = msg.status
    
    def load (self, commands):
        
        self.pause()
        rospy.loginfo ('Loading new set of commands')
        
        self.sync_lock.acquire()
        with self.lock:
            self.program = []
            i = 0
            for line in commands:
                i += 1
                parse = line.split()
                if not len (parse): continue
                
                try:
                    stmt = Statement (parse[0])
                    
                    if stmt.cmd == 'goal' or stmt.cmd == 'nav':
                        stmt.lat, stmt.lon = float(parse[1]), float(parse[2])
                    elif stmt.cmd == 'sleep':
                        stmt.msecs = int (parse[1])
                    
                    else:
                        raise ProgramFormatException ('unknown command: "{}"'.format(stmt.cmd))
                    
                    self.program.append (stmt)
                
                except Exception as e:
                    raise ProgramFormatException ('error in line {} of input [{}]'.format (i, e))
            
            #DEBUG
            for stmt in self.program:
                print stmt.__dict__
    
    def begin (self):
        self.sync_lock.acquire()
        with self.lock:
            rospy.loginfo ('Begin program execution')
            self.state = self.NEWCMD
    
    def pause (self):
        self.sync_lock.acquire()
        with self.lock:
            rospy.loginfo ('Pause program execution')
            self.state = self.PAUSE
    
    def resume (self):
        self.sync_lock.acquire()
        with self.lock:
            rospy.loginfo ('Resume program execution')
            self.state = self.RESUME
    
    def kill (self):
        self.sync_lock.acquire()
        with self.lock:
            rospy.loginfo ('Killing main thread')
            self.state = self.KILL
    
    def run (self):
        rate = rospy.Rate (self.freq)
        
        while True:
            rate.sleep()
            
            self.lock.acquire()
            try:
                if self.state == self.SLEEPING:
                    continue
                
                if self.state == self.NEWCMD:
                    # exec current statement
                    if not len (self.program):
                        rospy.loginfo ('End of Program')
                        self.state = self.SLEEPING
                        continue
                    self.stmt = self.program.pop(0)
                    
                if self.stmt.cmd == 'goal' or self.stmt.cmd == 'nav':
                
                    if self.state == self.NEWCMD or self.state == self.RESUME:
                        rospy.loginfo ('{} : {}, {}'.format (self.stmt.cmd, self.stmt.lat, self.stmt.lon))
                        avoid_ctrl = navigation.srv.AvoidNodeCtrlRequest.IGNORE_OBSTACLES
                        if self.stmt.cmd == 'nav':
                            avoid_ctrl = navigation.srv.AvoidNodeCtrlRequest.AVOID_OBSTACLES
                        
                        self.avoid_srv (avoid_ctrl, self.stmt.lat, self.stmt.lon)
                        self.plan_srv (pause=0, contin=0, rst=1)
                        self.plan_srv (pause=0, contin=1, rst=0)
                    
                    elif self.state == self.PAUSE or self.state == self.KILL:
                        self.avoid_srv (navigation.srv.AvoidNodeCtrlRequest.PAUSE,
                                self.stmt.lat, self.stmt.lon)
                        self.plan_srv (pause=1, contin=0, rst=0)
                    
                    elif self.plan_state == 1:
                        rospy.loginfo ('{} finished'.format (self.stmt.cmd))
                        self.avoid_srv (navigation.srv.AvoidNodeCtrlRequest.PAUSE,
                                self.stmt.lat, self.stmt.lon)
                        self.plan_srv (pause=0, contin=0, rst=1)
                        self.state = self.NEXT
                        self.plan_state = 0
                
                elif self.stmt.cmd == 'sleep':
                    if self.state == self.NEWCMD:
                        rospy.loginfo ('sleeping for {} msecs'.format (self.stmt.msecs))
                        self._sleepinc = 1000 / self.freq #millisecs
                        self._sleepmsec = 0
                    
                    if self.state != self.PAUSE and self.state != self.KILL:
                        if self._sleepmsec >= self.stmt.msecs:
                            rospy.loginfo ('sleep finished')
                            self.state = self.NEXT
                        self._sleepmsec += self._sleepinc
                
                if self.state == self.NEWCMD or self.state == self.RESUME:
                    self.state = self.RUNNING
                elif self.state == self.NEXT:
                    self.state = self.NEWCMD
                elif self.state == self.PAUSE:
                    self.state = self.SLEEPING
            
            except rospy.ServiceException as e:
                rospy.logerr ('error calling services')
                print e
            
            finally:
                self.lock.release()
                try:
                    self.sync_lock.release()
                except:
                    pass
                if self.state == self.KILL:
                    break
            # with end
        # loop end
        
    # func end

def run():
    node = GDMNode (FREQ)
    man = Manager (node)
    man.run()

if __name__=='__main__':
    run()
