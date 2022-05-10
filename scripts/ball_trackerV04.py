#!/usr/bin/env python3
# April 27th 2022
from __future__ import print_function
from cmath import log10
from cv2 import imshow
from matplotlib import markers
import rospy
from std_msgs.msg import Int32 # Messages used in the node must be imported.
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


import sys
import cv2
import numpy as np
import os
import math
import time

rospy.init_node("ball_tracker")


pub = rospy.Publisher('/sorter/camera/coords', Pose, queue_size=1000)
#sub	= rospy.Subscriber('image_topic_2', Image, queue_size=1000) #need CvBridge to proccess values
target_pose=Pose() # declaring a message variable of type Int32

## For image from file
# image_path = r'/home/sebas/eml4860/build/image2.png'
# directory = r'/home/sebas/eml4860/build'
# Timg = cv2.imread(image_path)
# img = Timg

## For image from USB camera
cap=cv2.VideoCapture(0)
_, img = cap.read()

#See if the raw image is looking right, check resolution of camera!
#cv2.imshow('read',Timg)
dimensions = img.shape
print("Dimensions (L,H): ",dimensions[1],"x",dimensions[0])

#Create scale from table dimensions to pixels (ex: table length/image length=scale)
tablex = 1.3462
tabley = 0.6604

x_scale = tablex/dimensions[1]
y_scale = tabley/dimensions[0]

y_offset = int(x_scale/0.2032)

# x_arm = int(dimensions[0]/2)
# y_arm = int(dimensions[1]/2)

x_arm = 173
y_arm = 247

#Arm outine
#img=cv2.circle(img,(x_arm),y_arm,50,(0,255,0),2)

print("x_scale: ",x_scale)
print("y_scale: ",y_scale)

#initialize coordinate variables
x_d=0.0
y_d=0.0
x_d_p=0.0
y_d_p=0.0

while(1):
    _, img = cap.read()
    #converting frame(img i.e BGR) to HSV (hue-saturation-value)
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    #Green balls
    blue_lower=np.array([82,48,92],np.uint8)
    blue_upper=np.array([97,221,207],np.uint8)
    #Pink sticky notes 
    pink_lower=np.array([171,85,194],np.uint8)
    pink_upper=np.array([180,160,255],np.uint8)

    blue=cv2.inRange(hsv,blue_lower,blue_upper)
    pink=cv2.inRange(hsv,pink_lower,pink_upper)
    #Morphological transformation, Dilation  	
    kernalB = np.ones((5 ,5), "uint8")
    kernalP = np.ones((5 ,5), "uint8")

    blue=cv2.dilate(blue,kernalB)
    pink=cv2.dilate(pink,kernalP)
    #cv2.imshow("Ball",blue)
    #cv2.imshow("Table Markers1",pink)

    # Get contours of table markers
    pinkTest = pink 
    timg = img
    (T, thresh) = cv2.threshold(pinkTest, 100, 255,
        cv2.THRESH_BINARY)
    contoursThresh, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(timg,contoursThresh, -1, (255,255,255), 1)
    cnt = contoursThresh

    coords = np.array([[0,0],[0,0],[0,0]])

    step = 0  
    for c in cnt:
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(timg,[box],0,(0,0,255),2)

        M = cv2.moments(c)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        #draw the center of the shape on the image
        cv2.circle(timg, (cx, cy), 7, (255, 255, 255), -1)
        coords[step,:] = cx,cy
        step = step + 1
        if step == 3:
            break
        cv2.putText(timg, "centroid", (cx - 20, cy - 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Do below with single contour around all sticky notes:
    # hull = cv2.convexHull(contoursThresh)

    print(coords)
    #cv2.imshow("Threshold Binary", thresh)
    #cv2.imwrite(os.path.join(directory, '3points.png'),thresh)
    #cv2.imshow("Marker Centroids", timg)

    # Locating table markers and their orientation: P3 is the far end of triangle

    L1 = math.dist([coords[0,0], coords[0,1]], [coords[1,0], coords[1,1]])
    L2 = math.dist([coords[1,0], coords[1,1]], [coords[2,0], coords[2,1]])
    L3 = math.dist([coords[2,0], coords[2,1]], [coords[0,0], coords[0,1]])
    base = min(L1,L2,L3)

    if L1 == base:
        p1 = np.array([coords[0,0], coords[0,1]])
        p2 = np.array([coords[1,0], coords[1,1]])
        p3 = np.array([coords[2,0], coords[2,1]])
    elif L2 == base:
        p1 = np.array([coords[1,0], coords[1,1]])
        p2 = np.array([coords[2,0], coords[2,1]])
        p3 = np.array([coords[0,0], coords[0,1]])
    else:
        p1 = np.array([coords[2,0], coords[2,1]])
        p2 = np.array([coords[0,0], coords[0,1]])
        p3 = np.array([coords[1,0], coords[1,1]])


    p13 = p1-p3
    p23 = p2-p3
    cross1 = np.cross(p13,p23)
    if cross1 < 0:
        print("markers found successfully.")
    else:
        temp = p1
        p1 = p2
        p2 = temp
        print("points swapped.")

    print ("P1, P2, P3 are: ", p1, p2, p3)

    cv2.putText(img,"point1 (start): ",(p1[0]-20,p1[1]+40),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120,255,255),1,cv2.LINE_AA)
    img=cv2.circle(img,(int(p1[0]),int(p1[1])),5,(255,0,0),-1)
    cv2.putText(img,"point3 (end, far): ",(p3[0]-20,p3[1]+40),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120,255,255),1,cv2.LINE_AA)
    img=cv2.circle(img,(int(p3[0]),int(p3[1])),5,(255,0,0),-1)
    #cv2.imshow("markers",img)

    #Tracking the Blue Color
    (contours,hierarchy)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    contours_poly = [None]*len(contours)
    boundRect = [None]*len(contours)
    centers = [None]*len(contours)
    radius = [None]*len(contours)

    for i, c in enumerate(contours):
        contours_poly[i] = cv2.approxPolyDP(c, 3, True)
        boundRect[i] = cv2.boundingRect(contours_poly[i])
        centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])

    # This is for sorting through to the max size contour;the only large ball. (i.e.can be altered) 
    if len(contours)>0:
        contour= max(contours,key=cv2.contourArea)
        #picking contour with max area on around the image
        area = cv2.contourArea(contour)
        
        if area>10: 
            x,y,w,h = cv2.boundingRect(contour)	
            img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            #draw rectangle around the ball on image
            x_cord = (2*x+w)/2
            y_cord = (2*y+h)/2
            img=cv2.circle(img,(int(x_cord), int (y_cord)),5,(255,0,0),-1)
            #Draws line from arm origin to ball
            img=cv2.line(img,((x_arm+y_offset),y_arm),(int((2*x+w)/2),int((2*y+h)/2)),(0,255,0),2)
        
            #Output in meters
            x_d= (((2*y+h)/2)) * x_scale
            y_d= (((2*x+w)/2)) * y_scale
            
            #Output in inches
            #x_d= (((2*y+h)/2)) * 0.04765625
            #y_d= (((2*x+w)/2)) * 0.047917

            s= 'x_d:'+ str(x_d)+ '   ' + 'y_d:'+str(y_d)
            
            cv2.putText(img,s,(x-20,y-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1,cv2.LINE_AA)
        
            if (abs(x_d-x_d_p)> 1 or abs(y_d-y_d_p)>1):
                target_pose.position.x=x_d
                target_pose.position.y=y_d
                target_pose.position.z=0.0
                pub.publish(target_pose)
            
                x_d_p=x_d
                y_d_p=y_d

            cv2.imshow("Ball Mask",blue)
            cv2.imshow("Color Tracking",img)

            if cv2.waitKey(1)== ord('q'):
                break
            time.sleep(0.1)
            
        
#cv2.imshow("Ball Mask",blue)
cv2.destroyAllWindows()
