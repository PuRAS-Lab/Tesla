#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple listener demo that listens to sensor_msgs/Image published 
## from the CARLA simulator on '/carla/ego_vehicle/camera/rgb/front/image_color' topic

import cv2
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

# Parameters
camera_info_topic = '/carla/ego_vehicle/camera/rgb/front/camera_info'
# Set in launch file:
# image_topic       = '/carla/ego_vehicle/camera/rgb/front/image_color'

# canny_threshold_1 = 100
# canny_threshold_2 = 200

# Global Variables
img_frame  = None
header     = None
img_height = 0
img_width  = 0
bridge     = None


def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255 # <-- This line altered for grayscale.
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image
    

# Check if Canny parameters are valid
def check_canny_params(canny_param1, canny_param2):
    if (canny_param1 > 0 and canny_param2 > 0) and (canny_param1 < canny_param2)  and (canny_param1 < 255 and canny_param2 < 255):
    	return True
    return False
 
# Perform Canny edge detector on image  
def canny_edge_detector(blur):
    # Defaul values
    canny_threshold_1 = 100
    canny_threshold_2 = 200
    
    if rospy.has_param('~canny_th_1') and rospy.has_param('~canny_th_2'):
    	canny_threshold_1 = int(rospy.get_param('~canny_th_1'))
    	canny_threshold_2 = int(rospy.get_param('~canny_th_2'))
    	if not check_canny_params(canny_threshold_1, canny_threshold_2):
    		canny_threshold_1 = 100	
    		canny_threshold_2 = 200
    
    edges = cv2.Canny(blur, canny_threshold_1, canny_threshold_2, apertureSize = 3)
    return edges

# Image processing callback      
def img_processing_callback(data):

    # Need add ROS parameters
    roi_x_min = 100
    roi_x_max = 400
    roi_y_max = 300
    
    roi_vertices = [
    (roi_x_min, img_height),
    (img_width / 2, roi_y_max),
    (roi_x_max, img_height)
    ]
    
    # Try to convert the ROS Image message to a CV2 Image
    try:
        global bridge
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    
    # Convert input RGB image to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
    # Remove white noise from image with low pass filter (Gaussian filter)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    
    # Canny edge detector
    edges = canny_edge_detector(blur)
    
    #Define Region of Interest
    roi = region_of_interest(edges, np.array([roi_vertices], np.int32))
    
    cv2.imshow("Image window blur, edge, roi", roi)

    cv2.waitKey(3)
    
# Camera basic information callback:    
def camera_info_callback(data):
    global img_height
    global img_width
    img_height =  data.height
    img_width  =  data.width

def carla_listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'carla_listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('carla_listener', anonymous=True)

    # Initialize the CvBridge class!
    global bridge
    bridge = CvBridge()
    
    # Get camera topic name from launch file
    if rospy.has_param('~camera_topic_name'):
    	image_topic = rospy.get_param('~camera_topic_name')
    	rospy.Subscriber(image_topic, Image, img_processing_callback)

    # Initalize a subscriber for getting camera basic information!
    rospy.Subscriber(camera_info_topic, CameraInfo, camera_info_callback)
    
    # Keep the program alive!
    rospy.spin()

if __name__ == '__main__':
    carla_listener()

