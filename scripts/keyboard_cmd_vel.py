#!/usr/bin/env python
<<<<<<< HEAD
#keyboard_cmd_vel.py
#Copyright (c) 2016 RT Corp. <shop@rt-net.jp>
#Copyright (c) 2016 Daisuke Sato <tiryoh@gmail.com>
#Copyright (c) 2016 Ryuichi Ueda <ryuichiueda@gmail.com>

#This software is released under the MIT License.
#http://opensource.org/licenses/mit-license.php

import rospy
import numpy as np
=======
import rospy
>>>>>>> 61d54b6ac011dece2546ccbc46b3b23e06237a74
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

rospy.wait_for_service('/motor_on')
rospy.wait_for_service('/motor_off')
<<<<<<< HEAD
rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
rospy.ServiceProxy('/motor_on',Trigger).call()
=======
rospy.on_shutdown(rospy.ServiceProxy('motor_off', Trigger).call)
rospy.ServiceProxy('/motor_on', Trigger).call()
>>>>>>> 61d54b6ac011dece2546ccbc46b3b23e06237a74

rospy.init_node('keyboard_cmd_vel')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
while not rospy.is_shutdown():
    vel = Twist()
<<<<<<< HEAD
    direction = raw_input('w: forward, s: backward, a: left, d: right, return: stop > ')
    if 'w' in direction: vel.linear.x = 0.15
    if 's' in direction: vel.linear.x = -0.15
    if 'a' in direction: vel.angular.z = np.pi/4
    if 'd' in direction: vel.angular.z = -np.pi/4
=======
    direction = raw_input('k : forward, j:backward, h:left, l:right, return: stop > ')
    if 'k' in direction: vel.linear.x = 0.15
    if 'j' in direction: vel.linear.x = -0.15
    if 'h' in direction: vel.angular.z = 3.14/4
    if 'l' in direction: vel.angular.z = -3.14/4
>>>>>>> 61d54b6ac011dece2546ccbc46b3b23e06237a74
    print vel
    pub.publish(vel)
