#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#스크립트를 실행할 Python 버전과 문자 인코딩을 지정

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
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

        rospy.Subscriber("/usb_cam/image_rect_color", Image, self.cam_CB)  # 'camera/rgb/image_raw/compressed' 토픽에서 CompressedImage 메시지를 구독하고 메시지가 수신되면 self.cam_CB 메서드를 호출

        self.original_window = "original_image"
        self.src_window = "src_point_control"
        self.dst_window = "dst_point_control"

        # HSV 범위를 조정하는 트랙바를 생성하는 데 필요한 변수와 설정을 초기화합니다.
        self.create_trackbar_flag = False  # 트랙바를 생성했는지 여부를 나타내는 플래그를 초기화합니다.
        self.cam_flag = False  # 이미지 메시지 수신 여부를 확인하는 변수를 초기화합니다.
        self.img_msg = Image()  # 이미지 메시지를 저장할 변수를 초기화합니다.

        # HSV 범위를 조정하는 트랙바의 초기값을 설정하는 변수들을 초기화합니다.
        # 이 코드는 해상도 640*480을 기준으로 작성되었습니다.
        self.img_x = 640
        self.img_y = 480

        ##src
        self.S_Upper_X_Value = 0  # left lower x 값을 초기화합니다.
        self.S_Upper_Y_Value = 0  # left down y 값을 초기화합니다.
        
        self.S_Lower_X_Value = 0  # left upper  을 초기화합니다.
        self.S_Lower_Y_Value = 0  # left up 을 초기화합니다.
        
        ##dst
        self.D_Upper_X_Value = 0  # left lower x 값을 초기화합니다.
        self.D_Upper_Y_Value = 0  # left down y 값을 초기화합니다.
        
        self.D_Lower_X_Value = 0  # left upper  을 초기화합니다.
        self.D_Lower_Y_Value = 0  # left up 을 초기화합니다.

        
    def create_trackbar_init(self, cv_img):
        # HSV 범위를 조절하기 위한 트랙바를 생성하는 메서드입니다.
        # 이 메서드는 트랙바를 사용하여 이미지 처리 매개변수를 조정할 수 있도록 초기 설정을 수행합니다.

        cv2.namedWindow(self.original_window, cv2.WINDOW_NORMAL)  # OpenCV 창을 생성하고 창 이름을 'original_image'로 설정합니다.
        cv2.namedWindow(self.src_window, cv2.WINDOW_NORMAL)  # OpenCV 창을 생성하고 창 이름을 'roi_image'로 설정합니다.
        cv2.namedWindow(self.dst_window, cv2.WINDOW_NORMAL)  # OpenCV 창을 생성하고 창 이름을 'roi_image'로 설정합니다.

        def hsv_track(value):
            # 트랙바 값이 변경될 때 호출되는 콜백 함수입니다. HSV 범위 트랙바의 현재 값을 업데이트합니다.
            
            ##src
            self.S_Upper_X_Value = cv2.getTrackbarPos("S_Upper_X", self.src_window)
            self.S_Upper_Y_Value = cv2.getTrackbarPos("S_Upper_Y", self.src_window)
            
            self.S_Lower_X_Value = cv2.getTrackbarPos("S_Lower_X", self.src_window)
            self.S_Lower_Y_Value = cv2.getTrackbarPos("S_Lower_Y", self.src_window)
            
            ##dst
            self.D_Upper_X_Value = cv2.getTrackbarPos("D_Upper_X", self.dst_window)
            self.D_Upper_Y_Value = cv2.getTrackbarPos("D_Upper_Y", self.dst_window)
            
            self.D_Lower_X_Value = cv2.getTrackbarPos("D_Lower_X", self.dst_window)
            self.D_Lower_Y_Value = cv2.getTrackbarPos("D_Lower_Y", self.dst_window)
    
    
        # 다양한 HSV 범위를 조정하기 위한 트랙바 생성
        ##src
        cv2.createTrackbar("S_Upper_X", self.src_window, 0, self.img_x // 2, hsv_track)  # H의 최소 임계 값 트랙바 생성
        cv2.createTrackbar("S_Upper_Y", self.src_window, 0, self.img_y // 2, hsv_track)  # H의 최소 임계 값 트랙바 생성
        
        cv2.createTrackbar("S_Lower_X", self.src_window, 0, self.img_x // 2, hsv_track)  # S의 최소 임계 값 트랙바 생성
        cv2.createTrackbar("S_Lower_Y", self.src_window, 0, self.img_y // 2, hsv_track)  # S의 최소 임계 값 트랙바 생성
        
        ##dst
        cv2.createTrackbar("D_Upper_X", self.dst_window, 0, self.img_x // 2, hsv_track)  # H의 최소 임계 값 트랙바 생성
        cv2.createTrackbar("D_Upper_Y", self.dst_window, 0, self.img_y // 2, hsv_track)  # H의 최소 임계 값 트랙바 생성
        
        cv2.createTrackbar("D_Lower_X", self.dst_window, 0, self.img_x // 2, hsv_track)  # S의 최소 임계 값 트랙바 생성
        cv2.createTrackbar("D_Lower_Y", self.dst_window, 0, self.img_y // 2, hsv_track)  # S의 최소 임계 값 트랙바 생성
        
        self.create_trackbar_flag = True  # 트랙바가 생성되었음을 표시합니다.

    def apply_bird_eyeview(self, cv_img):
        src_point1 = [0 + self.S_Lower_X_Value, self.img_y - self.S_Lower_Y_Value]
        src_point2 = [0 + self.S_Upper_X_Value, 0 + self.S_Upper_Y_Value]
        src_point3 = [self.img_x - self.S_Upper_X_Value, 0 + self.S_Upper_Y_Value]
        src_point4 = [self.img_x - self.S_Lower_X_Value, self.img_y - self.S_Lower_Y_Value]
        src_points = np.float32([src_point1, src_point2, src_point3, src_point4])

        dst_point1 = [0 + self.D_Lower_X_Value, self.img_y - self.D_Lower_Y_Value]
        dst_point2 = [0 + self.D_Upper_X_Value, 0 + self.D_Upper_Y_Value]
        dst_point3 = [self.img_x - self.D_Upper_X_Value, 0 + self.D_Upper_Y_Value]
        dst_point4 = [self.img_x - self.D_Lower_X_Value, self.img_y - self.D_Lower_Y_Value]
        dst_points = np.float32([dst_point1, dst_point2, dst_point3, dst_point4])

        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(cv_img, matrix, (cv_img.shape[1], cv_img.shape[0]))
        return warped_img
    
    def sense(self):
        # 이미지를 "감지(sense)"하는 함수입니다.
        img_msg = self.img_msg  # 수신한 이미지 메세지를 변수에 저장합니다.
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")  # CvBridge 모듈을 사용하여 이미지 메세지(Image)를 OpenCV 이미지로 변환합니다.

        if self.create_trackbar_flag == False:
            self.create_trackbar_init(cv_img)  # 트랙바 초기화 함수를 호출합니다.
        return cv_img  # 처리된 이미지를 반환합니다.

    def think(self, cv_img):
        # 이미지를 분석하고 "생각(think)"하는 함수입니다.
        warped_img = self.apply_bird_eyeview(cv_img)  # 이미지에서 마스크를 적용하여 HSV 이미지로 변환합니다.


        return warped_img

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
                warped_img = self.think(cv_img)  # 이미지 분석 및 "생각" 함수를 호출합니다.

                cv2.imshow(self.original_window, cv_img)  # 원본 이미지를 OpenCV 창에 표시합니다.
                cv2.imshow("warped_image", warped_img)  # 처리된 이미지를 OpenCV 창에 표시합니다.

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