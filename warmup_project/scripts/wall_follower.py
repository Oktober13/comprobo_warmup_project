#!/usr/bin/env python
""" Finds a wall, and drives along it. """

import rospy
import math
import atexit
import cv2

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from nav_msgs.msg import Odometry

rospy.init_node('receive_message')

class NeatoCallbacks(object):
	""" Useful callbacks for the neato. """
	def __init__(self):
		scan = rospy.Subscriber("/stable_scan", LaserScan, self.laserCallback)
		bump = rospy.Subscriber("/bump", Bump, self.bumpCallback)
		odom = rospy.Subscriber("/odom", Odometry, self.odomCallback)

		self.stopStatus = False
		self.minDistance = 10000000 # Infinity stand-in
		self.minAngle = 0
		self.pose = None
		return

	def laserCallback(self, msg):
		self.minDistance = 10000000; # Exact value doesn't matter- infinity
		# print msg.ranges

		average = sum(msg.ranges) / len(msg.ranges)
		print average

		cv2.HoughLines(msg.range, rho, 5, threshold[, lines[, srn[, stn]]])

		for angle in range(len(msg.ranges)):
			distance = msg.ranges[angle]
			# Prevent recognition of things too close to lidar, and of outliers
			if ((distance < self.minDistance) and (distance > 0.2)): 
				self.minAngle, self.minDistance = angle, distance
				# print "Angle: " + str(self.minAngle) + ", Distance: " + str(self.minDistance)
		return

	def bumpCallback(self, msg):
		if (msg.leftFront or msg.rightFront or msg.leftSide or msg.rightSide):
			self.stopStatus = True
		else:
			self.stopStatus = False
		return

	def odomCallback(self, msg):
		self.pose = msg.pose.pose.orientation
		return

class WallFollower(object):
	""" Finds the closest wall and follows it at a distance of 0.5 m. """
	def __init__(self):
		# rospy.init_node('test_subscribe')    # initialize ourselves with roscore
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
		self.r = rospy.Rate(10)

		self.desiredYaw = None
		self.vel = Twist(Vector3(0.05, 0.00, 0.00), Vector3(0.00, 0.00, 0.00))
		self.stop_vel = Twist(Vector3(0.00, 0.00, 0.00), Vector3(0.00, 0.00, 0.00))
		self.threshold = (math.pi) / 180
		self.pGain = 0.03 #Temp

		atexit.register(self.exit_handler)

	def findYaw(self, pose):
		# Convert quaternion to Euler yaw angle. https://www.vectornav.com/docs/default-source/documentation/vn-100-documentation/AN002.pdf?sfvrsn=19ee6b9_13
		if (pose != None):
			# return math.atan((2 * (pose.x * pose.y + pose.z * pose.w)) / ((pose.w ** 2.0) - (pose.z ** 2.0) - (pose.y ** 2.0) + (pose.x ** 2.0)))
			return math.atan2(pose.z, pose.w)
		else:
			return 0

	def findDesiredAngle(self, data, distance, tolerance):
		if (data.minDistance >= (distance + tolerance)): # When far away from wall, drive towards closest point on wall
			print "minAngle: " + str(data.minAngle)
			print data.minDistance
			return (0.5 * ((math.pi / 180) * data.minAngle)) - (math.pi / 2)
		elif (data.minDistance >= distance - tolerance):
			print "sleep"
			return -(math.pi / 4) + 0.1
		else:
			print "too close"
			return -(math.pi / 4) - 0.1

	def findError(self, data, desiredDistance):
		desiredAngle = self.findDesiredAngle(data, desiredDistance, 0.25)
		actualAngle = self.findYaw(data.pose)

		if (desiredAngle != None): self.desiredAngle = desiredAngle

		print "desiredAngle: " + str(desiredAngle) + ", " + "actualAngle: " + str(actualAngle)
		return (self.desiredAngle - actualAngle)

	def PID(self, currentVel, pGain, error):
		# Proportional, integrative, derivative control of the angular velocity
		P = pGain * error
		I = 0.0
		D = 0.0
		# print error
		return currentVel + P + I + D

	def followWall(self, data, desiredDistance):
		# Find yaw error and correct course by modifying angular and linear velocities
		yawError = self.findError(data, desiredDistance)

		if (abs(yawError) >= (2 * (math.pi / 18.0))): # Turn is prioritized when error in drive angle is >= +/-20 degrees
			self.vel.linear.x = 0.0
		else: # Error is acceptable: move forward
			self.vel.linear.x = 0.1

		self.vel.angular.z = self.PID(self.vel.angular.z, self.pGain, yawError)
		return

	def exit_handler(self):
		# Stop robot and exit gracefully
	    self.vel.linear.x, self.vel.angular.z = 0, 0
	    self.pub.publish(self.vel)
	    print "nope nope nope"
	    return

	def run(self, data):
		#While ros is running and bump sensor not triggered
		while ( ( not rospy.is_shutdown()) and (data.stopStatus == False) ):
			self.followWall(data, 0.5)
			# self.pub.publish(self.vel)
			self.r.sleep()
		self.pub.publish(self.stop_vel)
		return

if __name__ == '__main__':
	data = NeatoCallbacks()
	node = WallFollower()
	node.run(data)