#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from Lanes import FindLanes

class ImgCapture(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.image_sub = rospy.Subscriber("/pylon_camera_node/image_raw/",Image,self.camera_callback)
	self.findlines = FindLanes("calibrate_matrix.pickle")



    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
		cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
		img_RGB, cte, confidence_left,confidence_right = self.findlines.img_process(cv_image)
		usable = False
		if(confidence_left >=6 and confidence_right >=6):
			usable = True
		print('Usable:{}, CTE:{}, Left Confidence:{}, Right Confidence:{}'.format(usable, cte, confidence_left,confidence_right))
		
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", img_RGB)
        cv2.waitKey(1)


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
