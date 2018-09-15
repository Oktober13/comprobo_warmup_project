#!/usr/bin/env python

import rospy
import atexit
import math
import time

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

class SquareDrive(object):
	""" 
	Drives a neato in a 1m x 1m square.
	"""
	def __init__(self):
		squareSide = 1.0 # meters
		self.maxspeed = 0.5 # meters / second
		self.vel = Twist(Vector3(self.maxspeed, 0.00, 0.00), Vector3(0.00, 0.00, 0.00))
		self.sideNumber = 1
		self.turnNumber = 0
		self.sideTime = squareSide / self.maxspeed
		self.turnTime = (0.25 * math.pi) / self.maxspeed

		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
		self.r = rospy.Rate(0.1)
		self.startTime = time.time()
		atexit.register(self.exit_handler)

	def run(self):
		"""
		All possible states- run for...
		S1T0: 1 * sideTime + 0 * turnTime STRAIGHT
		S1T1: 1 * sideTime + 1 * turnTime TURN
		S2T1: 2 * sideTime + 1 * turnTime STRAIGHT
		S2T2: 2 * sideTime + 2 * turnTime TURN
		S3T2: 3 * sideTime + 2 * turnTime STRAIGHT
		S3T3: 3 * sideTime + 3 * turnTime TURN
		S4T3: 4 * sideTime + 3 * turnTime STRAIGHT
		S4T4: 4 * sideTime + 4 * turnTime TURN
		S5T4: STOP
		"""
		elapsed = time.time() - self.startTime
		# print elapsed

		if (elapsed <= 4 * (self.sideTime + self.turnTime)): # If all 4 sides haven't been traversed
			if (elapsed >= (self.sideNumber * self.sideTime) + (self.turnNumber * self.turnTime)): # Check if time for new action!
				if (self.sideNumber > self.turnNumber): # Just finished a side, perform a turn
					self.vel.linear.x = 0.0
					self.vel.angular.z = 2 * self.maxspeed
					self.turnNumber += 1
					print "turn"
				else:
					self.vel.linear.x = self.maxspeed
					self.vel.angular.z = 0.0
					self.sideNumber += 1
					print "straight"
		else:
			self.vel.linear.x = 0.0
			self.vel.angular.z = 0.0
			print "I'm here!"

		self.pub.publish(self.vel)

	def exit_handler(self):
	    self.vel.linear.x, self.vel.angular.z = 0, 0
	    self.pub.publish(self.vel)
	    print "walk on down the road"

if __name__ == '__main__':
	rospy.init_node('teleop')

	square = SquareDrive()

	while not rospy.is_shutdown():
		square.run()