#!/usr/bin/env python
#encording: utf8
import sys, rospy, math
from std_srvs.srv import Trigger, TriggerResponse

class Motor():
    def __init__(self): #initialization
        if not self.set_power(True): sys.exit(1)
        rospy.on_shutdown(self.set_power) #Switching the power of the motor
    	self.srv_on = rospy.Service('motor_on', Trigger, self.callback_on)
    	self.srv_off = rospy.Service('motor_off', Trigger, self.callback_off)
        self.last_time = rospy.Time.now()
        self.using_cmd_vel = False
    
    def set_power(self,onoff=False):
        en = "/dev/rtmotoren0"
        try:
            with open(en, 'w') as f:
                f.write("1\n" if onoff else "0\n")
            self.is_on = onoff
            return True
        except:
            rospy.logerr("cannot write to " + en)

        return False 

    def onoff_response(self, onoff):
	d = TriggerResponse()
	d.success = self.set_power(onoff)
	d.message = "ON" if self.is_on else "OFF"
	return d 
	
    def callback_on(self, message): return self.onoff_response(True)
    def callback_off(self, message): return self.onoff_response(False)

if __name__ == '__main__':
    rospy.init_node('motors')
    m = Motor()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()    

