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
	phase_n = 12
	direction = [		
		#[v_g, omega_g, duration]
		[0.0, 0.0, 3.0], 	#phase0
		[0.0, 180.0, 0.25], 	#phase1
		[0.0, 0.0, 3.0],	#phase2
		[0.2, 0.0, 1.5], 	#phase3
		[0.0, 0.0, 3.0], 	#phase4
		[0.0, -180.0, 0.5], 	#phase5
		[0.0, 0.0, 3.0],	#phase6
		[0.2, 0.0, 3.0],	#phase7
		[0.0, 0.0, 3.0], 	#phase8
		[0.0, 180.0, 0.50], 	#phase9
		[0.0, 0.0, 3.0],	#phase10
		[0.2, 0.0, 1.5],	#phase11
		[0.0, 0.0, 3.0], 	#phase12
		#[0.0, 180.0, 0.70], 	#phase13
		#[0.0, 0.0, 3.0],	#phase14
		#[0.2, 0.0, 2.0],	#phase15
		#[0.0, 0.0, 0.0]        #phase16
	]
	rate = rospy.Rate(freq)
	data = Twist()
	while not rospy.is_shutdown():
	
            #following_with_direction
	    if(phase != -2):
	        for i in range(phase_n):
		    if(phase == i and self.time-self.last_time >= direction[i][2]):
		        #if (i % 2 == 1):print("phase",str(i),"completed.")
			phase += 1	
		        v_g = direction[phase][0]
		        omega_g = direction[phase][1] * 3.14/180.0
		        self.last_time = self.time
			if(phase == phase_n): phase = -2

	    #switching
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
