#!/usr/bin/env python
import rospy,copy,sys
import numpy as np
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues, SwitchState, PulseCount
from subprocess import call
from subprocess import Popen

FREQ = 10 #Frequency[Hz]
CTR_P = 0.1 #Countrol Period[s]
DPP = 0.9 #Deg per Pulse[deg]
RPP = 0.0157 #Rad per Pulse[rad]
N = 6.98 #Number of Pulse for turn[-]
T = 9.2 #Tread[cm]
D = 4.6 #Diameter[cm]
R = 2.3 #Radious[cm]	

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
	self.theta = 0.0
	self.r = np.array([0.0,0.0])
	self.q = np.array([0.0,0.0])
	self.last_l = 0
	
	self.sensor_values = LightSensorValues()
	self.switch_states = SwitchState()
	self.pulse_count = PulseCount()
	self.last_pulse_count = PulseCount()
	rospy.Subscriber('/lightsensors', LightSensorValues, self.light_callback)
	rospy.Subscriber('/switch', SwitchState, self.switch_callback)
        rospy.Subscriber('/pulsecounter', PulseCount, self.pulse_callback)
    
    def calc_r(self):
	d_r = self.pulse_count.right - self.last_pulse_count.right
	d_l = self.pulse_count.left - self.last_pulse_count.left
	if(d_r != 0 and d_l != 0):
		omega_r = (d_r * RPP)/CTR_P
		omega_l = (d_l * RPP)/CTR_P
		v_r = omega_r * R
		v_l = omega_l * R
		v = float(v_l + v_r)/2.0
		omega = (v_l - v_r)/T
		self.theta += omega*CTR_P
		self.q[0] = np.cos(self.theta)
		self.q[1] = np.sin(self.theta)
		self.r += (v*CTR_P) * self.q
		self.last_pulse_count.right = self.pulse_count.right
		self.last_pulse_count.left = self.pulse_count.left
		return 1
	return 0
	
    def calc_v(self):
	l=((2*3.14*R)/N) * self.pulse_count.right	
	v=(l-self.last_l)/0.1
	self.last_l = l
	return v
 
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
	fb = 0

	rate = rospy.Rate(freq)
	data = Twist()
	while not rospy.is_shutdown():
	    if (self.switch_states.switch0):
		fb = self.calc_r()
		cnt += 1
		flag = 1
		#if (cnt%(freq * w_span)==0):
		if (fb): write_map(self.r[0], self.r[1], 'a')
	    	if (cnt%(freq * p_span)==0):
		    plot_map = Popen('./plot_xy.sh', shell=True)
		    cnt = 1

	    elif (flag == 1):
	        write_map(self.r[0], self.r[1], 'a')
	        plot_map = Popen('./plot_xy.sh', shell=True)
		cnt = 1
	        flag = 0
	        #print("imada!")
		
	    self.time += (1.0/freq)
	    #print(fb)
	    rate.sleep()

def reset_count():
    r = open('/dev/rtcounter_r1','w')
    r.write("0")
    r.close()
    l = open('/dev/rtcounter_l1', 'w')
    l.write("0")
    l.close()

if __name__ == '__main__':
    reset_count()
    init_map()
    rospy.init_node('mapping')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    #rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    #rospy.ServiceProxy('/motor_off', Trigger).call()
    Map().make()
