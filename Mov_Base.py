import math
import cv2 as cv
import numpy as np
import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped,Point,PointStamped,Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import subprocess
import os
import sys
import signal
from DronInteraction import VisualInteraction

class movement():
    def __init__(self,f):
        rospy.init_node('movement', anonymous=True)
        self.msg = PoseStamped()
        self.msg.header.frame_id='firefly/sensor_odometry1'
	self.pos_pub = rospy.Publisher("/posible_pose", Marker,latch=True,queue_size = 1)
        self.dir_pub = rospy.Publisher("/firefly/command/pose", PoseStamped,queue_size = 1)
        if f:
	    data = rospy.wait_for_message("/firefly/odometry_sensor1/pose", Pose, timeout=50.0)
            self.msg.pose=data
	    print self.msg

    def create_mark(self,x,y,z,delete):
	marker = Marker()
	marker.header.frame_id = "firefly/odometry_sensor1"
	marker.type = 0
	marker.ns = 'posible'
	if not delete:
		marker.action = marker.ADD
	else:
		marker.action = 3
	marker.scale.x = 0.2
	marker.scale.y = 0.2
	marker.scale.z = 0.2
	marker.color.a = 1.0
	marker.color.r = 0.0
	marker.color.g = 1.0
	marker.color.b = 0.0
	p1 = Point()
	p2 = Point()
	p2.x =x
	p2.z =z
	p2.y =y
	marker.points=[p1,p2]
	return marker

    def levantar(self):
        self.msg.pose.position.z = 2.0
	marker = self.create_mark(self.msg.pose.position.x,self.msg.pose.position.y,self.msg.pose.position.z,False)
	self.pos_pub.publish(marker)
	data = ""
	while data != "Ok"and data != "No":
		data = rospy.wait_for_message("/Output", String, timeout=5000.0).data
	if data != "No":
		marker = self.create_mark(0,0,2,True)
		self.pos_pub.publish(marker)
		self.dir_pub.publish(self.msg)
	else:
		print "Action Acepted"
		marker = self.create_mark(x,y,z,True)

    def dormir(self):
	marker = self.create_mark(self.msg.pose.position.x,self.msg.pose.position.y,self.msg.pose.position.z,False)
	self.pos_pub.publish(marker)
	data = ""
	while data != "Ok"and data != "No":
		data = rospy.wait_for_message("/Output", String, timeout=5000.0).data
	if data != "No":
		marker = self.create_mark(0,0,0,True)
		self.msg.pose.position.x = self.msg.pose.position.y = self.msg.pose.position.z = 0
		self.msg.pose.orientation.x = self.msg.pose.orientation.y = self.msg.pose.orientation.z = 0
		self.msg.pose.orientation.w = 1.0
		self.pos_pub.publish(marker)
		print "Action Acepted"
		self.dir_pub.publish(self.msg)
	else:
		print "Action Refused"
		marker = self.create_mark(x,y,z,True)

    def mover_abs(self,x,y,z):
	marker = self.create_mark(x,y,z,False)
	self.pos_pub.publish(marker)
	data = ""
	while data != "Ok" and data != "No":
		data = rospy.wait_for_message("/Output", String, timeout=5000.0).data
	if data != "No":
		marker = self.create_mark(x,y,z,True)
		self.msg.pose.position.x =x
		self.msg.pose.position.y = y
		self.msg.pose.position.z = z
		self.pos_pub.publish(marker)
		print "Action Acepted"
		self.dir_pub.publish(self.msg)
	else:
		print "Action Refused"
		marker = self.create_mark(x,y,z,True)

    def mover(self,x,y,z):
	data = rospy.wait_for_message("/firefly/odometry_sensor1/pose", Pose, timeout=50.0)
        self.msg.pose=data
	marker = self.create_mark(x,y,z,False)
	self.pos_pub.publish(marker)
	data = ""
	while data != "Ok"and data != "No":
		data = rospy.wait_for_message("/Output", String, timeout=5000.0).data
	if data != "No":
		marker = self.create_mark(x,y,z,True)
		self.msg.pose.position.x +=x
		self.msg.pose.position.y += y
		self.msg.pose.position.z += z
		self.pos_pub.publish(marker)
		print "Action Acepted"
		self.dir_pub.publish(self.msg)
	else:
		print "Action Refused"
		marker = self.create_mark(x,y,z,True)

global images

def callback(data):
    global images
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        images = cv.resize(image[0:480,0:640], None, fx=0.8, fy=0.8)
    except CvBridgeError as e:
        print(e)


Video = VisualInteraction()
bridge = CvBridge()
image_sub = rospy.Subscriber(name="/firefly/camera/camera_frontal/image_raw",data_class=Image, callback=callback,queue_size=1)
p = subprocess.Popen("exec " + "python Movimientos.py &", stdout=subprocess.PIPE, shell=True)
print p.pid
scale =[0.7]
Mov = movement(True)
Auto = False
while not rospy.is_shutdown():
	if Auto:
		data = rospy.wait_for_message("/firefly/odometry_sensor1/pose", Pose, timeout=5.0)
		while (abs(data.orientation.x)>0.0001 or abs(data.orientation.y)>0.0001):
		    data = rospy.wait_for_message("/firefly/odometry_sensor1/pose", Pose, timeout=5.0)
		image = images.copy()
		x_s, y_s, z_s, dir_r, canvas = Video.get_indication_from_images([image],scale)
		if x_s is not None and y_s is not None and z_s is not None:
			Mov.mover(0,int(-1*y_s),int(z_s))
	else:
		s = raw_input("l,d,m: ")
		if s == 'l':
			Mov.levantar()
		elif s == 'd':
		   	Mov.dormir()
		elif s == 'm':
		   	x = int(raw_input("x: "))
			y = -1*int(raw_input("y: "))
			z = int(raw_input("z: "))
			Mov.mover(x,y,z)
		else:
			break
os.killpg(os.getpgid(p.pid), signal.SIGTERM)
