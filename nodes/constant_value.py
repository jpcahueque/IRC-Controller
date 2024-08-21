#!/usr/bin/python

from re import T, X
from numpy.lib.polynomial import polysub
import rospy;import time;#import tf;
import math;import threading;import random
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from irc_controller.msg import RC
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np

valeurs_canaux=[200,500,100,500,500,500,500,500]

def callback_receive_throttle_data(msg):
	#rospy.loginfo("Message received : ")
	#rospy.loginfo(msg.data)
	valeurs_canaux[0] = msg.data

def callback_receive_rudder_data(msg):
	valeurs_canaux[1] = msg.data

def callback_receive_throttle_head_data(msg):
	valeurs_canaux[2] = msg.data



rospy.init_node('constant_value')

pub = rospy.Publisher('rc0', RC, queue_size=200)

sub_throttle = rospy.Subscriber("/throttle", Int16, callback_receive_throttle_data)

sub_rudder = rospy.Subscriber("/rudder", Int16, callback_receive_rudder_data)

sub_throttle_head = rospy.Subscriber("/throttle_head", Int16, callback_receive_throttle_head_data)

rate = rospy.Rate(500)  

if __name__ == '__main__':
 
	while not rospy.is_shutdown():

		pub.publish(valeurs_canaux) 
		
		rate.sleep()

	rospy.loginfo("Node was stopped")
3
