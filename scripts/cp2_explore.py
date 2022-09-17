#!/usr/bin/env python
import rospy,copy,sys
import numpy as np
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues, SwitchState
from subprocess import call
from subprocess import Popen

class Switch_master():   
    def __init__(self):
	self.states = SwitchState()
	self.sub = rospy.Subscriber('/switch', SwitchState, self.callback)
    def callback(self, message):
	self.states = message

class Explore():
    def __init__(self):
	self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	self.sensor_values = LightSensorValues()
	#self.f = open('myfile.txt','r')
	#self.data = self.f.read()
	#self.f.close()
	self.time = 0.0
	self.x = 0.0
	self.y = 0.0
	self.theta = 0.0

	self.s = Switch_master()
	rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)

    def callback(self, messages):
	self.sensor_values = messages

    def run(self):
	freq=10
	cnt=1
	write_interval = 1
	plot_interval=5
	d=0.1
	rate = rospy.Rate(freq)
	data = Twist()
	while not rospy.is_shutdown():
	    if (self.s.states.switch0) and (self.sensor_values.sum_all < 500):
		data.linear.x = 0.2
		self.x += d * np.cos(self.theta)
		self.y += d * np.sin(self.theta)
	        self.theta += 0.01
	        #cnt += 1
	        if(cnt%(freq*write_interval)==0):
	            write_map(self.x, self.y,'a')

	        #if(cnt%(freq*plot_interval)==0):
		   # plot_time = Popen('./plot_xy.sh', shell=True)
		   # cnt=1
	    else :
		data.linear.x = 0

	    
	    if(cnt%(freq*plot_interval)==0):
                plot_time = Popen('./plot_xy.sh', shell=True)
		cnt=1

	    self.time += (1.0/freq)
	    cnt += 1
	    #data.linear.x = 0.2 if self.sensor_values.sum_all < 500 else 0.0
	    self.cmd_vel.publish(data)
	    rate.sleep()

def init_map():
    write_map(0,0,'w')
    plot_time = Popen('./plot_xy.sh', shell=True)

def write_map(x,y,op):
    f=open('map2.csv',op)
    f.write(str(x)+ "," + str(y)+"\n")
    f.close()

class Switch_master():   
    def __init__(self):
	self.states = SwitchState()
	self.sub = rospy.Subscriber('/switch', SwitchState, self.callback)
    def callback(self, message):
	self.states = message

if __name__ == '__main__':
    init_map()
    rospy.init_node('wall_stop')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    rospy.ServiceProxy('/motor_off', Trigger).call()
    Explore().run()
