#!/usr/bin/env python

import tty
import select
import sys
import termios
import rospy
import atexit

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

class Teleop(object):
	""" 
	Teleoperation keyboard for a Neato- use arrow keys to navigate and SPACE bar to cancel all motion.
	Press key once to begin motion, and press twice to cancel that motion.
	"""
	def __init__(self):
		self.speedDic = {"1" : 0.1, "2" : 0.2, "3" : 0.3, "4" : 0.4, "5" : 0.5, "6" : 0.6, "7" : 0.7, "8" : 0.8, "9" : 0.9, "0" : 1.0,}
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
		self.vel = Twist(Vector3(0.00, 0.00, 0.00), Vector3(0.00, 0.00, 0.00))
		self.maxspeed = 0.1
		self.r = rospy.Rate(0.1)
		atexit.register(self.exit_handler)

	def getKey(self):
	    tty.setraw(sys.stdin.fileno())
	    select.select([sys.stdin], [], [], 0)
	    key = sys.stdin.read(1)
	    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

	    if ' ' in key:
	    	key = "stop"
	    elif '[' in key:
			key = self.getKey()
			if 'A' in key: key = "up"
			if 'B' in key: key = "down"
			if 'C' in key: key = "right"
			if 'D' in key: key = "left"
	    return key

	def interpKey(self, key):
		for value in self.speedDic:
			if value in key: 
				self.maxspeed = self.speedDic[value]
				print "speed is " + str(self.maxspeed)

		if key is "up":
			self.vel.linear.x = self.maxspeed if (self.vel.linear.x is 0) else 0
			print("forward")
		if key is "down":
			self.vel.linear.x = -self.maxspeed if (self.vel.linear.x is 0) else 0
			print("backward")
		if key is "right":
			self.vel.angular.z = -2 * self.maxspeed if (self.vel.angular.z is 0) else 0
			print("right")
		if key is "left":
			self.vel.angular.z = 2 * self.maxspeed if (self.vel.angular.z is 0) else 0
			print("left")
		if key is "stop":
			self.vel.linear.x, self.vel.angular.z = 0, 0
			print("stop... just stop")

		return self.vel

	def run(self):
		key = None

		while key != '\x03':
			key = self.getKey()
			if key is not None:
				self.interpKey(key)
			
			self.pub.publish(self.vel)

	def exit_handler(self):
	    self.vel.linear.x, self.vel.angular.z = 0, 0
	    self.pub.publish(self.vel)
	    print "thank you for shopping"

if __name__ == '__main__':
	rospy.init_node('teleop')
	settings = termios.tcgetattr(sys.stdin)

	teleop = Teleop()
	teleop.run()