#!/usr/bin/env python
import rospy,copy,sys
import numpy as np
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues, SwitchState, PulseCount
from subprocess import call
from subprocess import Popen

def init_map():
    write_map(0,0,'w')
    plot_time = Popen('./plot_xy.sh', shell=True)

def write_map(x,y,op):
    f=open('map2.csv',op)
    f.write(str(x)+ "," + str(y)+"\n")
    f.close()

class Map():
    def __init__(self):
	self.time = 0.0
	self.x = 0.0
	self.y = 0.0
	self.theta = 0
	
	self.sensor_values = LightSensorValues()
	self.switch_states = SwitchState()
	self.pulse_count = PulseCount()
	print(self.pulse_count)
	rospy.Subscriber('/lightsensors', LightSensorValues, self.light_callback)
	rospy.Subscriber('/switch', SwitchState, self.switch_callback)
	#rospy.Subscriber('/pulsecounter'. PulseCount, self.pulse_callback)
        rospy.Subscriber('/pulsecounter', PulseCount, self.pulse_callback)
	

    def light_callback(self,message):
	self.sensor_values = message
    def switch_callback(self,message):
	self.switch_states = message
    def pulse_callback(self,message):
	self.pulse_count = message

    def make(self):
	freq = 10
	cnt = 1
	w_span = 1
	p_span = 5
	d = 0.1
	flag = 0
	rate = rospy.Rate(freq)
	data = Twist()
	while not rospy.is_shutdown():
	    if (self.switch_states.switch0):
		self.x = float(self.pulse_count.right)/100.0
		self.x += 0.1
		#self.y += d * np.sin(self.theta)
		self.theta += 0.01
		cnt += 1
		if (cnt%(freq * w_span)==0):
		    write_map(self.x, self.y, 'a')
	    	if (cnt%(freq * p_span)==0):
		    plot_time = Popen('./plot_xy.sh', shell=True)
		    cnt = 1
		flag = 1
	    elif flag == 1:
		write_map(self.x, self.y, 'a')
		plot_map = Popen('./plot_xy.sh', shell=True)
	        flag = 0
		print("imada!")
		
	    self.time += (1.0/freq)
	    rate.sleep()

def reset_count():
    r = open('/dev/rtcounter_r0','w')
    r.write("0")
    r.close()
    l = open('/dev/rtcounter_l0', 'w')
    l.write("0")
    l.close()

if __name__ == '__main__':
    reset_count()
    init_map()
    rospy.init_node('wall_stop')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    rospy.ServiceProxy('/motor_off', Trigger).call()
    Map().make()
