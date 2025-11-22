#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class HSVExtractor:
    def __init__(self):
        rospy.init_node('hsv_extractor', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, self.image_callback)
        self.img_hsv = None
        self.lower_blue = None
        self.upper_blue = None

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            if self.img_hsv is not None:
                color_bgr = self.img_hsv[y, x]
                print("BGR Color:", color_bgr)
                print("HSV Color:", self.img_hsv[y, x])

    def run(self):
        cv.namedWindow('image')
        cv.setMouseCallback('image', self.mouse_callback)

        while not rospy.is_shutdown():
            if self.img_hsv is not None:
                cv.imshow('image', self.img_hsv)

            if cv.waitKey(1) & 0xFF == 27:
                break

        cv.destroyAllWindows()

if __name__ == "__main__":
    try:
        hsv_extractor = HSVExtractor()
        hsv_extractor.run()
    except rospy.ROSInterruptException:
        pass
