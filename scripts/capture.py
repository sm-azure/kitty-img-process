#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from Lanes import FindLanes

# Hyperparameters

# Confidence level
min_confidence = 3

class ImgCapture(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.image_sub = rospy.Subscriber("/pylon_camera_node/image_raw/",Image,self.camera_callback)
        self.ln_img_pub = rospy.Publisher("/kitt/img_process/image_raw/",Image, queue_size = 1)
        self.ln_img_undist_pub = rospy.Publisher("/kitt/img_process/undist/",Image, queue_size = 1)
        self.ln_img_crop_pub = rospy.Publisher("/kitt/img_process/undist/crop",Image, queue_size = 1)
        self.cte_pub = rospy.Publisher("/kitt/img_process/cte/", Int16 , queue_size = 1)
	self.findlines = FindLanes("calibrate_matrix.pickle")


    def resize(self, img, scale):
	return cv2.resize(img, (0,0), fx=scale, fy=scale)

    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
		cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
		img_RGB, cte, confidence_left,confidence_right, undistort_crop_img, undistort_img = self.findlines.img_process(cv_image)
		usable = False
		if(confidence_left >=min_confidence and confidence_right >=min_confidence):
			usable = True
		    	print('Usable:{}, CTE:{}, Left Confidence:{}, Right Confidence:{}'.format(usable, cte, confidence_left,confidence_right))
			self.cte_pub.publish(cte)
		image_message = self.bridge_object.cv2_to_imgmsg(self.resize(img_RGB,0.25), encoding="rgb8")
		image_undist_message = self.bridge_object.cv2_to_imgmsg(self.resize(undistort_img,0.25), encoding="rgb8")
		image_crop_message = self.bridge_object.cv2_to_imgmsg(self.resize(undistort_crop_img,0.25), encoding="rgb8")
		self.ln_img_pub.publish(image_message)
		self.ln_img_undist_pub.publish(image_undist_message)
		self.ln_img_crop_pub.publish(image_crop_message)
        except CvBridgeError as e:
            print(e)

        #cv2.imshow("Image window", img_RGB)
        #cv2.waitKey(1)
        # Send image to topic
        


def main():
    img_capture = ImgCapture()
    rospy.init_node('kitt_img_capture', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
