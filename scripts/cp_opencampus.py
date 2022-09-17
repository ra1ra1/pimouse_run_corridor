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
	self.time = 0.0
	self.last_time = 0.0
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
	v_g = 0.0
	omega_g = 0
	phase = -1
	rate = rospy.Rate(freq)
	data = Twist()
	while not rospy.is_shutdown():
 
	    if(phase == 0 and self.time-self.last_time >= 4.0):
		#phase1
		v_g=0.2
		omega_g=0.0
		self.last_time = self.time
		phase = 1

	    if(phase == 1 and self.time-self.last_time >= 2.5):
		#phase2
		v_g=0.0
		omega_g=0
		self.last_time = self.time
		phase = 2

	    if(phase == 2 and self.time-self.last_time >= 2.0):
		#phase3
		v_g=0.0 #0.2
		omega_g=150 * 3.14/180.0
		self.last_time = self.time
		phase = 3

	    if(phase == 3 and self.time-self.last_time >= 0.75):
		#phase4
		v_g=0.0
		omega_g=0.0
		self.last_time = self.time
		phase = 4

	    if(phase == 4 and self.time-self.last_time >= 2.0):
		#phase5
		v_g=0.2
		omega_g=0.0
		self.last_time = self.time
		phase = 5

	    if(phase == 5 and self.time-self.last_time >= 2.5):
		#phase6
		v_g=0.0
		omega_g=0.0
		self.last_time = self.time
		phase = -6

	    if(phase == 6 and self.time-self.last_time >= 2.0):
		#phase7
		v_g=0.2
		omega_g=0.0
		self.last_time = self.time
		phase = 7

	    if(phase == 7 and self.time-self.last_time >= 5.5):
		#phase8
		v_g=0.0
		omega_g=0.0
		self.last_time = self.time
		phase = -2

	    if (self.s.states.switch0) and (self.sensor_values.sum_all < 500):
		data.linear.x = v_g
		data.angular.z = omega_g
		if(phase == -1):phase=0
	    else :
		self.last_time = self.time
		data.linear.x = 0
		data.angular.z = 0
		phase = -1

	    self.cmd_vel.publish(data)
	    
	    #if(cnt%(freq*plot_interval)==0):
                #plot_time = Popen('./plot_xy.sh', shell=True)
		#cnt=1

	    #print(self.time - self.last_time)
	    self.time += (1.0/freq)
	    cnt += 1
	    #print(self.time - self.last_time)
	    #data.linear.x = 0.2 if self.sensor_values.sum_all < 500 else 0.0
	    #self.cmd_vel.publish(data

	    rate.sleep()

def init_map():
    write_map(0,0,'w')
    plot_time = Popen('./plot_xy.sh', shell=True)

def write_map(x,y,op):
    f=open('map2.csv',op)
    f.write(str(x)+ "," + str(y)+"\n")
    f.close()

if __name__ == '__main__':
    #init_map()
    rospy.init_node('wall_stop')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()
    Explore().run()
