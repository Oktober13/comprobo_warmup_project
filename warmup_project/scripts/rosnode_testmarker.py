#!/usr/bin/env python
""" This script publishes a marker to visualization_messages/Marker """
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
import rospy

class Sphere( object ):
	""" Creates a marker object"""
	def __init__( self, timestamp, position, color ):
		self.frame = 0
		self.header = Header( seq = self.frame, stamp = timestamp, frame_id = "odom")
		self.type = 2
		self.pose = Pose()
		self.color = color

		self.marker = Marker( 
			header = self.header,
			ns = "markers", 
			id = 0, 
			type = self.type,
			action = 0,
			pose = Pose( position = position, orientation = Quaternion( 0.0, 0.0, 0.0, 0.0 ) ),
			scale = Vector3( 1.0, 1.0, 1.0 ), # 1m diameter sphere of uniform radius
			color = self.color,
			lifetime = rospy.Time(0.0, 0.0),
			frame_locked = False,
			points = None,
			colors = None,
			text = None,
			mesh_resource = None,
			mesh_use_embedded_materials = None
		)


	def updateMarker( self, timestamp, marker_position, marker_color ):
		self.header = Header( seq = self.frame, stamp = timestamp, frame_id = "odom")
		self.pose = Pose( position = marker_position, orientation = Quaternion( 0.0, 0.0, 0.0, 0.0 ) )
		self.color = marker_color
		self.frame = self.frame + 1


class TestMarkerNode( object ):
	""" Publishes a message at 2 Hz, creating a marker at (1.0, 2.0, 0.0). """

	solid_red = ColorRGBA()
	solid_red.r = 1.0 # Opaque red
	solid_red.g = 0.0
	solid_red.b = 0.0
	solid_red.a = 1.0
	# solid_green = ColorRGBA( 0.0, 1.0, 0.0, 1.0 ), # Opaque green
	# solid_blue = ColorRGBA( 0.0, 0.0, 1.0, 1.0 ), # Opaque blue

	def __init__( self ):
		rospy.init_node('marker')
		self.pub = rospy.Publisher('/marker', Marker, queue_size = 10)
		self.r = rospy.Rate(0.1) # 10 Hz, or 10 cycles per second.

		self.sphere = Sphere( rospy.get_rostime(), Point( 1.0, 2.0, 0.0 ), self.solid_red )

	def setMarker( self ):
		self.sphere.updateMarker( rospy.get_rostime(), Point( 1.0, 2.0, 0.0 ), self.solid_red )
		print (self.sphere.marker)
		print ("")

		self.pub.publish( self.sphere.marker )

if __name__ == '__main__':
	node = TestMarkerNode()
	while not rospy.is_shutdown():
		node.setMarker()