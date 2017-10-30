import math
import cv2 as cv
import numpy as np
import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from DronInteraction import VisualInteraction,movement
from Config import *

global images
global count
#ImageCallback
def callback(data):
    global images
    global count
    if count is None:
	count=0
    if count ==0:
	    bridge = CvBridge()
	    try:
		image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
		image = cv.resize(image, None, fx=Res_scale, fy=Res_scale)
		images.append((image,data.header.seq))
		if len(images)> N_Images:
			images.pop(0)
	    except CvBridgeError as e:
		print(e)
	    count+=1
    else:
	count+=1
	if count >Skip_f:
		count=0



# Main Loop
count=0
images = []
Video = VisualInteraction()
bridge = CvBridge()
image_sub = rospy.Subscriber(name=Topic_camera,data_class=CompressedImage, callback=callback,queue_size=1)
Mov = movement(True)
pet_pub = rospy.Publisher("/Poseee",Image,queue_size = 1)
while not rospy.is_shutdown():
	while len(images) < N_Images:
		True
        image1,seq1 = images.pop(0)
	image2,seq2= images.pop(0)
        pet_pub.publish(bridge.cv2_to_imgmsg(image1))
        x_s, y_s, z_s, dir_r = Video.get_indication_from_images([image1,image2],scale)
	if Video.last_skul is not None and len(Video.last_skul) != 0 and Video.last_skul[0] is not None:
		canvas1 = Video.draw_skul(image1,Video.last_skul[0])
		if len(Video.last_skul) >1:
			canvas2 = Video.draw_skul(image2,Video.last_skul[1])
			canvas = cv.addWeighted(canvas1, 0.5, canvas2, 0.5, 0)
			pet_pub.publish(bridge.cv2_to_imgmsg(canvas))
		else:
			pet_pub.publish(bridge.cv2_to_imgmsg(canvas1))
	if x_s is not None and y_s is not None and z_s is not None and (x_s!= 0 or y_s != 0 or z_s != 0):
		x_s,y_s,z_s = Transform_Dir(x_s,y_s,z_s)
		x,y,z = Mov.mover(x_s,y_s,z_s)
		if x is not None:
			data = rospy.wait_for_message(Topic_Odometry, Odometry, timeout=5.0).pose.pose
			while (abs(data.position.x-x)>x_thres or abs(data.position.y-y)>y_thres or abs(data.position.z-z)>z_thres):
			    data = rospy.wait_for_message(Topic_Odometry, Odometry, timeout=5.0).pose.pose


