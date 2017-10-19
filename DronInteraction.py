import math
import cv2 as cv
import numpy as np
import CNN
import rospy
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
import speech_recognition as sr
from gtts import gTTS
import os
import time
import pygame


class VisualInteraction():
    def __init__(self):
        self.SKUL = CNN.skeleton()

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
            print("Elapsed time: %0.10f seconds." % elapsed_time)
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
        return x,y,z,dir,canvas

    def directions(self,dir):
        a_r, z_r = dir
        x=y=z=0.0
        if z_r > 30.0:
            x = 1.0
        else:
            x = 0.0
        angle = a_r[2]
        if angle >= -20.0 and angle <= 30.0:
            y = -1
        elif angle > 30.0 and angle <= 60.0:
            y = -1
            z = 1
        elif angle > 60.0 and angle <= 110.0:
            z = 1
        elif angle > 110.0 and angle <= 150.0:
            y = 1
            z = 1
        elif angle < -20.0 and angle >= -70.0:
            y = -1
            z = -1
        elif angle < -70.0 and angle >= -120.0:
            z = -1
        elif angle < -120.0 and angle >= -160.0:
            z = -1
            y = 1
        else:
            y = 1
        return x,y,z


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
        if Skul is None:
            print "Skeleton not found"
            return None, None, None, None,None
        if not ('hand right' in Skul.keys() and 'hand left' in Skul.keys()
            and 'shoulder left' in Skul.keys() and 'shoulder right' in Skul.keys()):
            print "One or more joints are missing"
            return None,None,None,None,None
        x,y,z,dir_r = self.calc(Skul,True)
        self.last_skul = Skul
        return x,y,z,dir_r,None

    def get_cuadrante_mov_multi(self,image,scales):
        Skul = self.obtain_skeleton(image,True,scales)
        x=y=z=0
        i=0
        dir = []
        for Skel,canvas in Skul:
            if Skel is None or canvas is None:
                print "Skeleton not found"
                return None,None,None,None
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
            self.last_skul = Skel
        x =math.ceil(x/len(image))
        y=math.ceil(y*1.0/len(image))
        z=math.ceil(z*1.0/len(image))
        return x,y,z,dir



