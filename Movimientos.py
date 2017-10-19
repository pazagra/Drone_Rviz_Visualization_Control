#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from geometry_msgs.msg import PointStamped,PoseStamped,Pose


topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray)

rospy.init_node('register')



MARKERS_MAX = 8

while not rospy.is_shutdown():
   markerArray = MarkerArray()
   data = rospy.wait_for_message("/firefly/odometry_sensor1/position", PointStamped, timeout=5.0)
   for y in xrange(-1,2,1):
	for z in xrange(-1,2,1):
	   pos=[0.0,y,z]
	   marker = Marker()
	   marker.header.frame_id = "firefly/odometry_sensor1"
	   marker.type = marker.SPHERE
	   marker.action = marker.ADD
	   marker.scale.x = 0.2
	   marker.scale.y = 0.2
	   marker.scale.z = 0.2
	   marker.color.a = 1.0
	   marker.color.r = 0.0
	   marker.color.g = 0.0
	   marker.color.b = 1.0
	   marker.pose.orientation.w = 1.0
	   marker.pose.position.x = pos[0]
	   marker.pose.position.y = pos[1]
	   marker.pose.position.z = pos[2]

	   markerArray.markers.append(marker)
   id = 0
   for m in markerArray.markers:
       m.id = id
       id += 1

   publisher.publish(markerArray)


   rospy.sleep(0.01)
