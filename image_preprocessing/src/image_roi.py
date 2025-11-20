#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#스크립트를 실행할 Python 버전과 문자 인코딩을 지정

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from morai_msgs.msg import CtrlCmd
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
from math import *
from time import *

# 필요한 Python 모듈과 ROS 라이브러리를 임포트

class ImageProcessor:
    def __init__(self):
        # ImageProcessor 클래스의 초기화 메서드

        self.bridge = CvBridge()  # ROS 이미지와 OpenCV 이미지 간 변환을 위한 CvBridge 인스턴스 생성

        rospy.init_node("image_roi_node", anonymous=True)  # ROS 노드를 초기화하고 이름을 'image_hsv_node'로 설정
        
        rospy.Subscriber("/usb_cam/image_rect_color", CompressedImage, self.cam_CB)  # 'camera/rgb/image_raw/compressed' 토픽에서 CompressedImage 메시지를 구독하고 메시지가 수신되면 self.cam_CB 메서드를 호출

        self.original_window = "original_image"
        self.roi_window = "roi_control"

        # HSV 범위를 조정하는 트랙바를 생성하는 데 필요한 변수와 설정을 초기화합니다.
        self.create_trackbar_flag = False  # 트랙바를 생성했는지 여부를 나타내는 플래그를 초기화합니다.
        self.cam_flag = False  # 이미지 메시지 수신 여부를 확인하는 변수를 초기화합니다.
        self.img_msg = CompressedImage()  # 이미지 메시지를 저장할 변수를 초기화합니다.

        # HSV 범위를 조정하는 트랙바의 초기값을 설정하는 변수들을 초기화합니다.
        # 이 코드는 해상도 1280*720을 기준으로 작성되었습니다.
        self.img_x = 1280
        self.img_y = 720

        self.Upper_X_Value = 0  # left lower x 값을 초기화합니다.
        self.Upper_Y_Value = 0  # left down y 값을 초기화합니다.
        
        self.Lower_X_Value = 0  # left upper  을 초기화합니다.
        self.Lower_Y_Value = 0  # left up 을 초기화합니다.
        


        
    def create_trackbar_init(self, cv_img):
        # HSV 범위를 조절하기 위한 트랙바를 생성하는 메서드입니다.
        # 이 메서드는 트랙바를 사용하여 이미지 처리 매개변수를 조정할 수 있도록 초기 설정을 수행합니다.

        cv2.namedWindow(self.original_window, cv2.WINDOW_NORMAL)  # OpenCV 창을 생성하고 창 이름을 'original_image'로 설정합니다.
        cv2.namedWindow(self.roi_window, cv2.WINDOW_NORMAL)  # OpenCV 창을 생성하고 창 이름을 'roi_image'로 설정합니다.

        def hsv_track(value):
            # 트랙바 값이 변경될 때 호출되는 콜백 함수입니다. HSV 범위 트랙바의 현재 값을 업데이트합니다.
            self.Upper_X_Value = cv2.getTrackbarPos("Upper_X", self.roi_window)
            self.Upper_Y_Value = cv2.getTrackbarPos("Upper_Y", self.roi_window)
            
            self.Lower_X_Value = cv2.getTrackbarPos("Lower_X", self.roi_window)
            self.Lower_Y_Value = cv2.getTrackbarPos("Lower_Y", self.roi_window)
            


        # 다양한 HSV 범위를 조정하기 위한 트랙바 생성
        cv2.createTrackbar("Upper_X", self.roi_window, 0, self.img_x // 2, hsv_track)  # H의 최소 임계 값 트랙바 생성
        cv2.createTrackbar("Upper_Y", self.roi_window, 0, self.img_y // 2, hsv_track)  # H의 최소 임계 값 트랙바 생성
        
        cv2.createTrackbar("Lower_X", self.roi_window, 0, self.img_x // 2, hsv_track)  # S의 최소 임계 값 트랙바 생성
        cv2.createTrackbar("Lower_Y", self.roi_window, 0, self.img_y // 2, hsv_track)  # S의 최소 임계 값 트랙바 생성
    
        self.create_trackbar_flag = True  # 트랙바가 생성되었음을 표시합니다.

    def apply_roi(self, cv_img):
        # BGR 이미지를 HSV 이미지로 변환합니다.
        region = np.array([[
            (0 + self.Lower_X_Value, self.img_y - self.Lower_Y_Value),
            (0 + self.Upper_X_Value, 0 + self.Upper_Y_Value),
            (self.img_x - self.Upper_X_Value, 0 + self.Upper_Y_Value),
            (self.img_x - self.Lower_X_Value, self.img_y - self.Lower_Y_Value),
        ]])
        
        mask = np.zeros_like(cv_img)
        mask = cv2.fillPoly(mask, region, (255, 255, 255))
        mask = cv2.bitwise_and(cv_img, mask)
        return mask  # 마스크를 적용한 이미지를 반환합니다.
    
    def sense(self):
        # 이미지를 "감지(sense)"하는 함수입니다.
        img_msg = self.img_msg  # 수신한 이미지 메세지를 변수에 저장합니다.
        cv_img = self.bridge.compressed_imgmsg_to_cv2(img_msg)  # CvBridge 모듈을 사용하여 이미지 메세지(CompressedImage)를 OpenCV 이미지로 변환합니다.

        if self.create_trackbar_flag == False:
            self.create_trackbar_init(cv_img)  # 트랙바 초기화 함수를 호출합니다.
        return cv_img  # 처리된 이미지를 반환합니다.

    def think(self, cv_img):
        # 이미지를 분석하고 "생각(think)"하는 함수입니다.
        roi_img = self.apply_roi(cv_img)  # 이미지에서 마스크를 적용하여 HSV 이미지로 변환합니다.


        return roi_img

    def cam_CB(self, msg):
        # 이미지 메세지를 받아오기 위한 콜백 함수입니다.
        if msg != -1:
            self.img_msg = msg  # 유효한 메세지인 경우, 메세지를 self.img_msg로 저장합니다.
            self.cam_flag = True  # 이미지 메세지 수신 확인 변수를 True로 설정합니다.
        else:
            self.cam_flag = False  # 이미지 메세지 수신 확인 변수를 False로 설정합니다.


    def run(self):
        # 메인 루프 함수로 이미지 처리 및 제어 동작이 수행됩니다.
        if self.cam_flag:
            cv_img = self.sense()  # 이미지를 감지하고 가져옵니다.

            if self.create_trackbar_flag:
                roi_img = self.think(cv_img)  # 이미지 분석 및 "생각" 함수를 호출합니다.

                cv2.imshow(self.original_window, cv_img)  # 원본 이미지를 OpenCV 창에 표시합니다.
                cv2.imshow("roi_image", roi_img)  # 처리된 이미지를 OpenCV 창에 표시합니다.

                cv2.waitKey(1)  # 이미지 표시를 위한 대기시간입니다.

def main():
    try:
        image_processor = ImageProcessor()
        while not rospy.is_shutdown():
            # ImageProcessor 인스턴스를 생성하고 run 메서드를 호출하여 프로그램을 실행합니다.
            image_processor.run()
            
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()  # 주요 프로그램이 시작됩니다.