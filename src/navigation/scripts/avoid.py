#!/usr/bin/env python
"""
Obstacle Avoidance

Subscribes to a ThetaR.msg representing visible obstacles
Also subscribes to present location (Point.msg)
and target location (Point.msg)

Publishes temporary destination Point
"""

import rospy
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs

import navigation.msg
import navigation.srv

import numpy as np
import scipy.ndimage
from rospy.numpy_msg import numpy_msg
import threading

# need not occur synchronously
# worst case theoretical delay from input to output is additive over these two freqs
COMPUTE_FREQ = 50
PUBLISH_FREQ = 50

COLLISION_RADIUS = 0.8 # in metres
OBSTACLE_LIMIT = 10 # range of detection, in metres

class VolatileVars:

    # modes of operation
    IGNORE_OBSTACLES = 1
    AVOID_OBSTACLES = 2
    DO_NOTHING = 0
    SHUTDOWN = -1
    # for printing purposes only
    MODE_STRING = {IGNORE_OBSTACLES: 'IGNORE', AVOID_OBSTACLES: 'AVOID', DO_NOTHING: 'PAUSED', SHUTDOWN: 'EXIT_PENDING'}

    def __init__ (self):
        self.lat, self.lon = (0, 0)
        self.goal_lat, self.goal_lon = (0.1, 0)
        # from -180 to 180 degrees
        self.yaw = 0
        
        self.map = np.arange(0, dtype=np.float64)
        #self.map = np.array ([10, 10, 10, np.nan, np.inf, np.inf, np.inf, 8, 9, 5, np.inf, np.inf, np.inf, np.inf]) 
        self.range = 10
        
        self.map_lock = threading.Lock()
        self.curr_lock = threading.Lock()
        self.goal_lock = threading.Lock()
        self.imu_lock = threading.Lock()
        self.inp_locks = self.map_lock, self.curr_lock, self.goal_lock, self.imu_lock
        
        # to publish
        self.dist = np.inf
        self.deviation = 0 # angle
        self.goal_dist = np.inf
        self.out_lock = threading.Lock()
        
        # program control variable
        self.control_lock = threading.Lock()
        self.program_ctrl = self.DO_NOTHING
        
        #DEBUG)
        self.valid_map = np.zeros (0, dtype=np.bool_)
    
    # all update and set funcitons are thread safe
    def update_map (self, msg):
        with self.map_lock:
            #self.map = msg.data
            #self.range = msg.range
            # TODO subscribe to lidar
            self._plot_update_map (msg)
    
    def _plot_update_map (self, msg):
        self.map = msg.data
        self.range = msg.range
    
    def update_curr_loc (self, msg):
        with self.curr_lock:
            self.lat = msg.latitude
            self.lon = msg.longitude
    
    def update_goal_loc (self, msg):
        with self.goal_lock:
            self.goal_lat = msg.latitude
            self.goal_lon = msg.longitude
    
    def update_yaw (self, msg):
        with self.imu_lock:
            self.yaw = msg.yaw
    
    def set_output (self, dist, deviation, goal_dist, valid_map=np.zeros(0, dtype=np.bool_)):
        with self.out_lock:
            self.dist = dist
            self.deviation = deviation
            self.goal_dist = goal_dist
            
            #DEBUG
            self.valid_map = valid_map
    
    def get_output (self):
        with self.out_lock:
            return self.dist, self.deviation, self.goal_dist, self.valid_map
    
    def lock_inputs (self):
        for lock in self.inp_locks:
            lock.acquire()
    
    def unlock_inputs (self):
        for lock in self.inp_locks:
            lock.release()

# Handles most ROS functionality
class AvoidNode:

    # create a 2-way dict to map ROS constants to VolatileVars constants
    _ros = navigation.srv.AvoidNodeCtrlRequest
    _vv = VolatileVars
    MODE_DICT = {_ros.IGNORE_OBSTACLES:_vv.IGNORE_OBSTACLES,
                 _ros.AVOID_OBSTACLES:_vv.AVOID_OBSTACLES,
                 _ros.PAUSE:_vv.DO_NOTHING}
    for key in MODE_DICT.keys():
        MODE_DICT [MODE_DICT[key]] = key

    def __init__ (self, vvars, pub_freq):
        self.vvars = vvars
        self.pub_freq = pub_freq
        
        #rospy.Subscriber ("obs_map", numpy_msg(navigation.msg.ObstacleMap), vvars.update_map)
        # TODO subscribe to lidar
        ## temporary
        rospy.Subscriber ("obs_map", numpy_msg(navigation.msg.ObstacleMap), vvars.update_map)
        ##
        rospy.Subscriber ("curr_loc", sensor_msgs.NavSatFix, vvars.update_curr_loc)
        rospy.Subscriber ("goal_loc", sensor_msgs.NavSatFix, vvars.update_goal_loc)
        rospy.Subscriber ("imu", sensor_msgs.Imu, vvars.update_yaw)
        self.pub = rospy.Publisher ("target", navigation.msg.Target, queue_size=1)
        rospy.init_node ('avoid_node')
        
        #DEBUG
        self.valid_pub = rospy.Publisher ('valid_map', numpy_msg(std_msgs.ByteMultiArray), queue_size=1)
        
        # service to control operating mode
        rospy.Service ('avoid_node_ctrl', navigation.srv.AvoidNodeCtrl, self.prog_ctrl_server)
        
        rospy.loginfo ('AvoidNode setup complete')
        rospy.loginfo ('AvoidNode Operating mode: ' + VolatileVars.MODE_STRING [self.vvars.program_ctrl])
    
    # server to change the operating mode (self.vvars.program_ctrl)
    def prog_ctrl_server (self, msg):
        ret = None
        with self.vvars.control_lock:
            try:
                self.vvars.program_ctrl = self.MODE_DICT [msg.mode]
            except KeyError:
                if msg.mode != msg.EMPTY:
                    rospy.logwarn ('avoid_node_ctrl called with invalid value {}'.format (msg.mode))
            
            if all (~np.isnan ([msg.latitude, msg.longitude])):
                self.vvars.update_goal_loc (msg)
            
            ret = navigation.srv.AvoidNodeCtrlResponse (self.MODE_DICT [self.vvars.program_ctrl])
            rospy.loginfo ('AvoidNode Operating mode: ' + VolatileVars.MODE_STRING [self.vvars.program_ctrl])
        
        return ret
    
    # run in main thread
    def run_publish (self):
        rate = rospy.Rate (self.pub_freq)
        while not rospy.is_shutdown():
            if self.vvars.program_ctrl != VolatileVars.DO_NOTHING:
                msg = navigation.msg.Target()
                valid_msg = std_msgs.ByteMultiArray()
                msg.target_dist, msg.deviation, msg.goal_dist, valid_msg.data = self.vvars.get_output()
                self.pub.publish (msg)
                self.valid_pub.publish (valid_msg)
            rate.sleep()

class Compute:
    def __init__ (self, vvars, freq, coll_rad, obs_lim):
        self.vvars = vvars
        self.FREQ = freq
        self.COLL_RAD = coll_rad
        self.OBS_LIM = obs_lim
        
        # least count of expansion angle (see calc_valid_angles)
        self.ANGLE_LC = 5 * np.pi / 180 # in radians
        self.EARTH_RADIUS = 6371 * 1000 # in metres
        
        # DEBUG flags
        debug_collision = True  # closest obstacle is closer than min. distance
        debug_stuck = True      # no valid angles to target
    
    # calculates distance and bearing (degrees)
    def great_circle (self, lat1, lon1, lat2, lon2):
        phi1, phi2, lambda1, lambda2 = map (np.radians, [lat1, lat2, lon1, lon2])
        dphi = phi2 - phi1
        dlambda = lambda2 - lambda1
        
        a = np.sin(dphi/2)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(dlambda/2)**2
        c = 2 * np.arctan2 (a**0.5, (1-a)**0.5)
        dist = self.EARTH_RADIUS * c
        
        theta = np.arctan2 (np.sin(dlambda)*np.cos(phi2), np.cos(phi1)*np.sin(phi2) - np.sin(phi1)*np.cos(phi2)*np.cos(dlambda))
        bearing = np.degrees(theta)
        
        return dist, bearing
        
    def calc_valid_angles (self, data, goal_dist, limit=np.inf):
        # computational complexity proportional to 90 / self.ANGLE_LC
        
        delta = 2 * np.pi / data.size
        invalid = np.zeros (shape=data.shape, dtype=np.bool_)
        
        # increase the required clearance angle (theta) in steps and eliminate relevant distances
        # 2*span + 1 is the number of indices occupied by the expanded obstacle
        span = 0
        for theta in np.arange (self.ANGLE_LC, np.pi/2, self.ANGLE_LC):
            
            with np.errstate (divide='ignore'):
                min_rad = self.COLL_RAD / np.sin (span * delta) # span=0 should result in np.inf
            
            # update span AFTER calculating min_rad (be generous with obs. expansion)
            span = int (theta / delta)
            
            # don't consider obstacles farther away than limit
            # can't 'continue' here, make sure that obstacles between limit and next min_rad are caught
            if (min_rad > limit):
                min_rad = limit
            
            with np.errstate (invalid='ignore'):
                t_invalid = data < min_rad # np.nan shold evaluate to false
            
            weights = np.ones (2 * span + 1)
            t_invalid = scipy.ndimage.convolve (t_invalid, weights, mode='wrap')
            
            invalid = np.logical_or (invalid, t_invalid)
        
        invalid = np.logical_or (invalid, np.isnan(data))
        return np.logical_not (invalid)
    
    # not thread safe (acqurie input locks before calling)
    # calculates distance to target and deviation from current yaw angle
    # also calculates distance to goal
    def calc (self):
        
        # all angles w.r.t yaw (unless specified otherwise)
        
        # alpha is the 'as the crow flies' angle to the goal
        goal_dist, bearing = self.great_circle (self.vvars.lat, self.vvars.lon,
                self.vvars.goal_lat, self.vvars.goal_lon)
        alpha = bearing - self.vvars.yaw
        
        data = self.vvars.map    # shallow copy
        if not (data.size > 0):
            return goal_dist, alpha, goal_dist, np.zeros(0, dtype=np.bool_)
        
        i = np.arange(data.size)    # array of indices
        theta = 360.0 * i / data.size   # 0-360 degrees
        
        # delta is the array of deviation angles from alpha
        delta = np.mod(np.abs(theta - alpha), 360)
        delta[delta > 180] = 360 - delta[delta > 180]
        
        # minimise delta over all valid directions
        valid = self.calc_valid_angles (data, goal_dist, limit=self.OBS_LIM)
        try:
            min_subidx = np.argmin(delta[valid])
            self.debug_stuck = False
        except:
            # no valid angles
            self.debug_stuck = True
            return 0, 0, goal_dist, valid
        min_idx = i[valid][min_subidx]
        
        # deviation is the required angle to turn (reminder: all angles wrt yaw)
        deviation = theta[min_idx]
        if deviation > 180:
            deviation -= 360    # -180 to 180 degrees
        
        # decide whether to target periphery of vision or inner point
        dist = self.vvars.range
        if goal_dist < self.vvars.range:
            variation = (deviation-alpha) % 360
            tolerance = 360.0 / data.size
            if variation < tolerance or 360 - variation < tolerance:
                dist = goal_dist
        
        return dist, deviation, goal_dist, valid
    
    # not thread safe (acquire locks before calling)
    # calculates distance and required turning angle to the goal point
    # ignores obstacle map
    def no_obs_calc (self):
        dist, bearing = self.great_circle (self.vvars.lat, self.vvars.lon,
                self.vvars.goal_lat, self.vvars.goal_lon)
        deviation = bearing - self.vvars.yaw
        if deviation > 180:
            deviation -= 360
        return dist, deviation
    
    # can be run in separate thread
    def run (self):
    
        rate = rospy.Rate (self.FREQ)
        while True:
            ctrl = self.vvars.program_ctrl
            if ctrl == VolatileVars.SHUTDOWN:
                return
            
            dist, deviation, goal_dist, valid_map = 0, 0, 0, np.zeros(0, dtype=np.bool_)
            self.vvars.lock_inputs()
            
            if ctrl == VolatileVars.AVOID_OBSTACLES:
                dist, deviation, goal_dist, valid_map  = self.calc()
            
            elif ctrl == VolatileVars.IGNORE_OBSTACLES:
                goal_dist, _ = dist, deviation = self.no_obs_calc()
            
            self.vvars.unlock_inputs()
            self.vvars.set_output (dist, deviation, goal_dist, valid_map)
            rate.sleep()

def run():
    vvars = VolatileVars()
    node = AvoidNode(vvars, PUBLISH_FREQ)
    compute = Compute(vvars, freq=COMPUTE_FREQ,
                             coll_rad=COLLISION_RADIUS,
                             obs_lim=OBSTACLE_LIMIT)
    cthread = threading.Thread (target=compute.run)
    cthread.start()
    node.run_publish()
    vvars.program_ctrl = vvars.SHUTDOWN
    print

run()
