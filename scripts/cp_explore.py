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
	frq=10
	cnt=0
	interval=5
	rate = rospy.Rate(frq)
	data = Twist()
	while not rospy.is_shutdown():
	    if (self.s.states.switch0) and (self.sensor_values.sum_all < 500):
		data.linear.x = 0.2
		self.x += 1 * np.cos(self.theta)
		self.y += 1 * np.sin(self.theta)
	    else :
		data.linear.x = 0

	    self.time += (1.0/frq)
	    cnt+=1

	    f_map=open('map.csv','a')
            f_map.write(str(self.time) + "," + str(self.x) + "," + str(self.y) +"\n")
	    f_map.close()
	    #print(self.time,self.x)

	    if(cnt%(frq*interval)==0):
		plot_time=Popen('./plot_time.sh', shell=True)
		cnt=0
	    #data.linear.x = 0.2 if self.sensor_values.sum_all < 500 else 0.0
	    self.cmd_vel.publish(data)
	    rate.sleep()

def init_map():
    f=open('map.csv','w')
    f.write("")
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
