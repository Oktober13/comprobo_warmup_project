#!/usr/bin/env python
""" Finds a wall, and drives along it. """

import rospy
import math
import atexit
import statistics

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from nav_msgs.msg import Odometry
from rosnode_testmarker import RVizMarker
from visualization_msgs.msg import Marker

rospy.init_node('receive_message')

class NeatoCallbacks(object):
	""" Useful callbacks for the neato. """
	def __init__(self):
		scan = rospy.Subscriber("/scan", LaserScan, self.laserCallback)
		bump = rospy.Subscriber("/bump", Bump, self.bumpCallback)
		odom = rospy.Subscriber("/odom", Odometry, self.odomCallback)

		self.stopStatus = False
		self.position = None
		self.orientation = None
		self.person = {}
		return

	def laserCallback(self, msg):
		average = sum(msg.ranges) / len(msg.ranges) + 0.25

		for angle in range(len(msg.ranges)):
			distance = msg.ranges[angle]
			# Person will be closer to the Neato than most things.
			if ((distance < average) and (distance != 0.0)):
				self.person[angle] = distance
		return

	def bumpCallback(self, msg):
		if (msg.leftFront or msg.rightFront or msg.leftSide or msg.rightSide):
			self.stopStatus = True
		else:
			self.stopStatus = False
		return

	def odomCallback(self, msg):
		self.orientation = msg.pose.pose.orientation
		self.position = msg.pose.pose.position
		return

class PersonFollower(object):
	""" Finds the closest wall and follows it at a distance of 0.5 m. """
	def __init__(self):
		self.velPub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
		self.vizPub = rospy.Publisher('/marker', Marker, queue_size = 10)
		self.r = rospy.Rate(10)

		self.vel = Twist(Vector3(0.05, 0.00, 0.00), Vector3(0.00, 0.00, 0.00))
		self.stopVel = Twist(Vector3(0.00, 0.00, 0.00), Vector3(0.00, 0.00, 0.00))

		atexit.register(self.exitHandler)

	def findYaw(self, quaternion):
		# Convert quaternion to Euler yaw angle.
		if (quaternion != None):
			return math.atan2(quaternion.z, quaternion.w)
			# return (degrees) * (180 / math.pi)
		else:
			return 0

	def angle_normalize(self, z):
		""" convenience function to map an angle to the range [-pi,pi] """
		return math.atan2(math.sin(z), math.cos(z))

	def angle_diff(self, a, b):
		""" Calculates the difference between angle a and angle b (both should be in radians)
			the difference is always based on the closest rotation from angle a to angle b
			examples:
				angle_diff(.1,.2) -> -.1
				angle_diff(.1, 2*math.pi - .1) -> .2
				angle_diff(.1, .2+2*math.pi) -> -.1
		"""
		a = self.angle_normalize(a)
		b = self.angle_normalize(b)
		d1 = a-b
		d2 = 2*math.pi - math.fabs(d1)
		if d1 > 0:
			d2 *= -1.0
		if math.fabs(d1) < math.fabs(d2):
			return d1
		else:
			return d2

	def followPerson(self, data, yaw):
		# Follows a large obstruction of non-zero distance in a relatively clear space
		if (data.person != {}):
			personAngle = self.angle_normalize(statistics.mean(data.person.keys()))
			difference = self.angle_diff(personAngle, yaw)

			print str(personAngle) + ", yaw: " + str(yaw)
			if(difference < 0.0):
				# print str(statistics.mean(data.person.keys())) + ", yaw: " + str(yaw)
				print "right"
				self.vel.angular.z = -0.5
			if(difference > 0.0):
				print "left"
				self.vel.angular.z = 0.5
			if(difference == 0.0):
				print "front"
				self.vel.angular.z = 0.0

			if (self.vel.linear.x != 0):
				self.vel.linear.x = 0.1 * (statistics.mean(data.person.values()) / max(data.person.values()))
			return personAngle

	def visualizer(self, desiredAngle, data):
		solid_red = ColorRGBA()
		solid_red.r, solid_red.g, solid_red.b, solid_red.a = 1.0, 0.0, 0.0, 1.0 # Opaque red
		solid_blue = ColorRGBA()
		solid_blue.r, solid_blue.g, solid_blue.b, solid_blue.a = 0.0, 0.0, 1.0, 1.0 # Opaque blue

		if (desiredAngle is not None) and (data.position is not None):
			theta = desiredAngle
			person_arrow = RVizMarker(rospy.get_rostime(), None, solid_blue, 0, Vector3( 0.1, 0.2, 0.2 ), [data.position, Point( data.position.x + math.cos(theta), data.position.y + math.sin(theta), 0.0 )] )
			self.vizPub.publish( person_arrow.marker )

	def exitHandler(self):
		# Stop robot and exit gracefully
	    self.velPub.publish(self.stopVel)
	    print "nope nope nope"
	    return

	def run(self, data):
		#While ros is running and bump sensor not triggered
		while ( ( not rospy.is_shutdown()) and (data.stopStatus == False) ):
			yaw = self.findYaw(data.orientation)
			personAngle = self.followPerson(data, yaw)
			self.visualizer(personAngle, data)
			self.velPub.publish(self.vel)
			self.r.sleep()
		self.velPub.publish(self.stopVel)
		return

if __name__ == '__main__':
	data = NeatoCallbacks()
	node = PersonFollower()
	node.run(data)