import math
import cv2 as cv
import numpy as np
import CNN
import rospy
import roslib
import os
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped,Point,PointStamped,Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from Config import *


class VisualInteraction():
    def __init__(self):
        self.SKUL = CNN.skeleton()
	self.last_skul = None

    def obtain_skeleton(self,image,multiple,scales):
	start_time = time.time()
	Skeleton = []
        if multiple:
            Skeleton = self.SKUL.get_skeleton_multiple(image,scales)
            elapsed_time = time.time() - start_time
    	    print("Skeleton time: %0.2f seconds." % elapsed_time)
            return Skeleton
        else:
            canvas,Skeleton = self.SKUL.get_skeleton(image[0],scales)
            elapsed_time = time.time() - start_time
            print("Elapsed time: %0.2f seconds." % elapsed_time)
            return Skeleton,canvas

    def obtain_z_angle(self,skeleton,xy_l,xy_r):
        if 'hand left' in skeleton.keys() and 'arm left' in skeleton.keys() and 'shoulder left' in skeleton.keys() and 'shoulder right' in skeleton.keys():
            gamma_l = 0.0
            dist = math.hypot(skeleton['shoulder left'][0] - skeleton['shoulder right'][0],
                              skeleton['shoulder left'][1] - skeleton['shoulder right'][1])
            dist_l = math.hypot(skeleton['arm left'][0] - skeleton['hand left'][0],
                                skeleton['arm left'][1] - skeleton['hand left'][1])
            if dist !=0:
                perc_l = float(dist_l) / float(dist) if (float(dist_l) / float(dist)) <= 1.0 else 1.0
                angle_l = 90.0 * (1.0+ gamma_l - (perc_l))
            else:
                angle_l = None
        else:
            angle_l = None
        if 'hand right' in skeleton.keys() and 'arm right' in skeleton.keys() and 'shoulder left' in skeleton.keys() and 'shoulder right' in skeleton.keys():
            gamma_r = 0.0
            dist = math.hypot(skeleton['shoulder left'][0] - skeleton['shoulder right'][0],
                              skeleton['shoulder left'][1] - skeleton['shoulder right'][1])
            dist_r = math.hypot(skeleton['arm right'][0] - skeleton['hand right'][0],
                                skeleton['arm right'][1] - skeleton['hand right'][1])
            if dist != 0:
                perc_r = float(dist_r) / float(dist) if (float(dist_r) / float(dist)) <= 1.0 else 1.0
                angle_r = 90.0 * (1.0+ gamma_r - (perc_r))
            else:
                angle_r = None
        else:
            angle_r = None
        return angle_l,angle_r

    def obtain_xy_angle(self,skeleton):
        def plot_point(point, angle, length):
            # unpack the first point
            x, y = point

            # find the end point
            endy = y + length * math.sin(math.radians(angle))
            endx = x + length * math.cos(math.radians(angle))
            return (int(endx), int(endy))

        if 'hand left' in skeleton.keys() and 'arm left' in skeleton.keys():
            angle = np.rad2deg(math.atan2(skeleton['hand left'][0] - skeleton['arm left'][0],
                                          skeleton['hand left'][1] - skeleton['arm left'][1]))
            angle2 = np.rad2deg(math.atan2(skeleton['shoulder right'][0] - skeleton['shoulder left'][0],
                                          skeleton['shoulder right'][1] - skeleton['shoulder left'][1]))
            if angle2 <= 0.0:
                angle=angle-(180.0-abs(angle2))
            else:
                angle = angle + (180.0 - abs(angle2))
            if angle >= 0.0:
                angle = angle-180.0
            else:
                angle = 180.0+angle
            start = (skeleton['hand left'][1], skeleton['hand left'][0])
            end = plot_point(start, angle, 200)
            left= (start,end,angle)
        else:
            left = (None,None,None)
        if 'hand right' in skeleton.keys() and 'arm right' in skeleton.keys():
            angle = np.rad2deg(math.atan2(skeleton['hand right'][0] - skeleton['arm right'][0],
                                          skeleton['hand right'][1] - skeleton['arm right'][1]))
            angle2 = np.rad2deg(math.atan2(skeleton['shoulder right'][0] - skeleton['shoulder left'][0],
                                          skeleton['shoulder right'][1] - skeleton['shoulder left'][1]))
            if angle2 <= 0.0:
                angle=angle-(180.0-abs(angle2))
            else:
                angle = angle + (180.0 - abs(angle2))
            if angle >= 0.0:
                angle = angle-180.0
            else:
                angle = 180.0+angle
            start = (skeleton['hand right'][1], skeleton['hand right'][0])
            end = plot_point(start, angle, 200)
            right = (start, end,angle)
        else:
            right = (None,None,None)
        return left,right

    def obtain_direction(self,skeleton):
        a_l, a_r= self.obtain_xy_angle(skeleton)
        z_l,z_r = self.obtain_z_angle(skeleton,a_l[2],a_r[2])
        return (a_l,z_l),(a_r,z_r)

    def get_indication_from_images(self,images,scales):
        if len(images)>1:
            x,y,z,dir = self.get_cuadrante_mov_multi(images,scales)
        else:
            x, y, z,dir,canvas = self.get_cuadrante_mov_single(images,scales,None)
        return x,y,z,dir

    def directions(self,dir):
        a_r, z_r = dir
        x=y=z=0.0
        if z_r > 30.0:
            x = 1.0
        else:
            x = 0.0
        angle = a_r[2]
        if Angles_values[0] <= angle <= Angles_values[1]:
            y = -1
        elif Angles_values[1] < angle <= Angles_values[2]:
            y = -1
            z = 1
        elif Angles_values[2] < angle <= Angles_values[3]:
            z = 1
        elif Angles_values[3] < angle <= Angles_values[4]:
            y = 1
            z = 1
        elif Angles_values[0] > angle >= Angles_values[5]:
            y = -1
            z = -1
        elif Angles_values[5] > angle >= Angles_values[6]:
            z = -1
        elif Angles_values[6] > angle >= Angles_values[7]:
            z = -1
            y = 1
        else:
            y = 1
        return x,y,z

    def extract_hand(self,skel,dir,right):
        def plot_point(point, angle, length):
            # unpack the first point
            x, y = point

            # find the end point
            endx = x + length * math.sin(math.radians(angle))*-1.0
            endy = y + length * math.cos(math.radians(angle))*-1.0
            return (int(endy), int(endx))
        if right:
            v = np.array(skel.values())
            lenght =( min(v.max(0)[1]+30,480) - max(v.min(0)[1]-30,0) ) *0.1
            center = plot_point(skel["hand right"],dir[0][2],lenght)
            init = (int(center[0]-lenght),int(center[1]-lenght))
            finit = (int(center[0]+lenght), int(center[1]+lenght))
            return init,finit
        else:
            v = np.array(skel.values())
            lenght =( min(v.max(0)[1]+30,480) - max(v.min(0)[1]-30,0) ) *0.4
            center = plot_point(skel["hand left"],dir[0][2],lenght)
            init = (center[0] - lenght, center[1] - lenght)
            finit = (center[0] + lenght, center[1] + lenght)
            return init, finit

    def calc(self,Skel,right):
        dir_l, dir_r = self.obtain_direction(Skel)
        dist_r = math.hypot(Skel['chest'][0] - Skel['hand right'][0],
                            Skel['chest'][1] - Skel['hand right'][1])
        dist_l = math.hypot(Skel['chest'][0] - Skel['hand left'][0],
                            Skel['chest'][1] - Skel['hand left'][1])
        if dist_r > dist_l or right:
            x,y,z=self.directions(dir_r)
        else:
            x,y,z=self.directions(dir_l)
        return x,y,z,dir_r

    def get_cuadrante_mov_single(self,image,scales,Skul):
        if Skul is None:
            Skul,canvas= self.obtain_skeleton(image,False,scales)
	else:
	    canvas = None
        if Skul is None:
            print "Skeleton not found"
            return None, None, None, None,None
        if not ('hand right' in Skul.keys() and 'hand left' in Skul.keys()
            and 'shoulder left' in Skul.keys() and 'shoulder right' in Skul.keys()):
            print "One or more joints are missing"
            return None,None,None,None,None
        x,y,z,dir_r = self.calc(Skul,True)
        self.last_skul = Skul
        return x,y,z,dir_r,canvas

    def draw_skul(self,image,skul):
	c = self.SKUL.draw_skeleton(image,skul)
	return c

    def get_cuadrante_mov_multi(self,image,scales):
        Skul = self.obtain_skeleton(image,True,scales)
        x=y=z=0
        i=0
        dir = []
	self.last_skul = []
        for Skel,canvas in Skul:
            if Skel is None or canvas is None:
                print "Skeleton not found"
                continue
            if not ('hand right' in Skel.keys() and 'hand left' in Skel.keys()
                and 'shoulder left' in Skel.keys() and 'shoulder right' in Skel.keys()):
                continue
            x_s,y_s,z_s,dir_r = self.calc(Skel,True)
            dir.append(dir_r)
            x += x_s
            y+=y_s
            z+=z_s
            i+=1
            print x
            print y
            print z
            self.last_skul.append(Skel)
	if len(self.last_skul)==0:
		return None,None,None,dir
        x =math.floor(x/len(self.last_skul))
        y=math.floor(y*1.0/len(self.last_skul))
        z=math.floor(z*1.0/len(self.last_skul))
        return x,y,z,dir



class movement():
    def __init__(self,f):
        rospy.init_node('movement', anonymous=True)
        self.msg = PoseStamped()
        self.msg.header.frame_id=Frame_id_mov
	self.pet_pub = rospy.Publisher("/Asking",String,queue_size = 1)
	self.pos_pub = rospy.Publisher("/posible_pose", Marker,latch=True,queue_size = 1)
        self.dir_pub = rospy.Publisher(Topic_command, PoseStamped,queue_size = 1)
        if f:
	    data = rospy.wait_for_message(Topic_Odometry,Odometry, timeout=50.0).pose
            self.msg.pose=data.pose
	    print self.msg

    def create_mark(self,x0,y0,z0,x,y,z,delete):
	marker = Marker()
	marker.header.frame_id = Frame_id_world
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
	p1.x =x0
	p1.y =y0
	p1.z =z0
	p2 = Point()
	p2.x =x
	p2.z =z
	p2.y =y
	marker.points=[p1,p2]
	return marker

    def mover(self,x,y,z):
	data = rospy.wait_for_message(Topic_Odometry, Odometry, timeout=5.0).pose
        self.msg.pose.position=data.pose.position
	try:
		data = rospy.wait_for_message("/Output", String, timeout=0.01).data
	except :
		True
	marker = self.create_mark(self.msg.pose.position.x,self.msg.pose.position.y,self.msg.pose.position.z,self.msg.pose.position.x+x,self.msg.pose.position.y+y,self.msg.pose.position.z+z,False)
	self.pos_pub.publish(marker)
	data = ""
	while data != "Ok"and data != "No":
		data = rospy.wait_for_message("/Output", String, timeout=5000.0).data
	if data != "No":
		marker = self.create_mark(self.msg.pose.position.x,self.msg.pose.position.y,self.msg.pose.position.z,self.msg.pose.position.x+x,self.msg.pose.position.y+y,self.msg.pose.position.z+z,True)
		self.msg.pose.position.x +=x
		self.msg.pose.position.y += y
		self.msg.pose.position.z += z
		if self.msg.pose.position.x >= x_max:
			self.msg.pose.position.x = x_max
		elif self.msg.pose.position.x <= x_min:
			self.msg.pose.position.x = x_min
		if self.msg.pose.position.y >= y_max:
			self.msg.pose.position.y = y_max
		elif self.msg.pose.position.y <= y_min:
			self.msg.pose.position.y = y_min
		if self.msg.pose.position.z >= z_max:
			self.msg.pose.position.z = z_max
		elif self.msg.pose.position.z <= z_min:
			self.msg.pose.position.z = z_min
		self.pos_pub.publish(marker)
		print "Action Acepted"
		self.dir_pub.publish(self.msg)
		return self.msg.pose.position.x,self.msg.pose.position.y,self.msg.pose.position.z
	else:
		print "Action Denied"
		marker = self.create_mark(self.msg.pose.position.x,self.msg.pose.position.y,self.msg.pose.position.z,self.msg.pose.position.x+x,self.msg.pose.position.y+y,self.msg.pose.position.z+z,True)
		self.pos_pub.publish(marker)
		return None,None,None																																											


