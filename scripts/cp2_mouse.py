#!/usr/bin/env python
#motors.py
#Copyright (c) 2016 Ryuichi Ueda <ryuichiueda@gmail.com>
#This software is released under the MIT License.
#http://opensource.org/licenses/mit-license.php

import sys, rospy, math, tf
import numpy as np
from pimouse_ros.msg import MotorFreqs, PulseCount
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.srv import TimedMotion
from nav_msgs.msg import Odometry

FREQ = 10 #Frequency[Hz]
CTR_P = 0.1 #Countrol Period[s]
DPP = 0.9 #Deg per Pulse[deg]
RPP = 0.0157 #Rad per Pulse[rad]
N = 6.98 #Number of Pulse for turn[-]
T = 9.2 #Tread[cm]
D = 4.6 #Diameter[cm]
R = 2.3 #Radious[cm]	

class Mouse():
    def __init__(self):
        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0
	self.flag=0

        self.pulse_count = PulseCount()
	self.last_pulse_count = PulseCount()
        rospy.Subscriber('/pulsecounter', PulseCount, self.pulse_callback)

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time

    def send_odom(self):
        self.cur_time = rospy.Time.now()
        #dt = self.cur_time.to_sec() - self.last_time.to_sec()
        #self.x += self.vx * math.cos(self.th) * dt
        #self.y += self.vx * math.sin(self.th) * dt
        #self.th += self.vth * dt 
        self.calc_r()
        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.bc_odom.sendTransform((self.x,self.y,0.0), q, self.cur_time,"base_link","odom")

        odom = Odometry()
        odom.header.stamp = self.cur_time
        odom.header.frame_id = "odom"

        odom.pose.pose.position = Point(self.x,self.y,0)
        odom.pose.pose.orientation = Quaternion(*q)

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        self.pub_odom.publish(odom)
        self.last_time = self.cur_time

    def calc_r(self):
        d_t = self.cur_time.to_sec() - self.last_time.to_sec()
	d_r = self.pulse_count.right - self.last_pulse_count.right
	d_l = self.pulse_count.left - self.last_pulse_count.left
	if(d_r != 0 and d_l != 0):
	    omega_r = (d_r * RPP)/d_t
	    omega_l = (d_l * RPP)/d_t
	    v_r = omega_r * R
	    v_l = omega_l * R
	    v = float(v_l + v_r)/2.0 * 0.01 # cm -> m
	    omega = (v_r - v_l)/T
	    self.th += omega*d_t
            self.x += (v*d_t)* np.cos(self.th)
            self.y += (v*d_t)* np.sin(self.th)
	    self.last_pulse_count.right = self.pulse_count.right
	    self.last_pulse_count.left = self.pulse_count.left
	    return 1
	return 0

    def pulse_callback(self,message):
	self.pulse_count = message

if __name__ == '__main__':
    rospy.init_node('mouse')
    m = Mouse()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        m.send_odom()
        rate.sleep()
