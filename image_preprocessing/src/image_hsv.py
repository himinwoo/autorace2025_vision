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

        rospy.init_node("image_hsv_node", anonymous=True)  # ROS 노드를 초기화하고 이름을 'image_hsv_node'로 설정
        
        rospy.Subscriber("/usb_cam/image_rect_color", Image, self.cam_CB)  # 'camera/rgb/image_raw/compressed' 토픽에서 Image 메시지를 구독하고 메시지가 수신되면 self.cam_CB 메서드를 호출

        self.original_window = "original_image"
        self.hsv_window = "hsv_image"
        self.bin_window = "bin_image"

        # HSV 범위를 조정하는 트랙바를 생성하는 데 필요한 변수와 설정을 초기화합니다.
        self.create_trackbar_flag = False  # 트랙바를 생성했는지 여부를 나타내는 플래그를 초기화합니다.
        self.cam_flag = False  # 이미지 메시지 수신 여부를 확인하는 변수를 초기화합니다.
        self.img_msg = Image()  # 이미지 메시지를 저장할 변수를 초기화합니다.

        # HSV 범위를 조정하는 트랙바의 초기값을 설정하는 변수들을 초기화합니다.
        self.L_H_Value = 0  # HSV 색상(Hue)의 하한(Hue, Saturation, Value)을 초기화합니다.
        self.L_S_Value = 0  # HSV 채도(Saturation)의 하한을 초기화합니다.
        self.L_V_Value = 0  # HSV 밝기(Value)의 하한을 초기화합니다.
        self.U_H_Value = 0  # HSV 색상의 상한을 초기화합니다.
        self.U_S_Value = 0  # HSV 채도의 상한을 초기화합니다.
        self.U_V_Value = 0  # HSV 밝기의 상한을 초기화합니다.
        
    def create_trackbar_init(self, cv_img):
        # HSV 범위를 조절하기 위한 트랙바를 생성하는 메서드입니다.
        # 이 메서드는 트랙바를 사용하여 이미지 처리 매개변수를 조정할 수 있도록 초기 설정을 수행합니다.

        cv2.namedWindow(self.original_window, cv2.WINDOW_NORMAL)  # OpenCV 창을 생성하고 창 이름을 'original_image'로 설정합니다.
        cv2.namedWindow(self.hsv_window, cv2.WINDOW_NORMAL)  # OpenCV 창을 생성하고 창 이름을 'hsv_image'로 설정합니다.

        def hsv_track(value):
            # 트랙바 값이 변경될 때 호출되는 콜백 함수입니다. HSV 범위 트랙바의 현재 값을 업데이트합니다.
            self.L_H_Value = cv2.getTrackbarPos("Low_H", self.hsv_window)
            self.L_S_Value = cv2.getTrackbarPos("Low_S", self.hsv_window)
            self.L_V_Value = cv2.getTrackbarPos("Low_V", self.hsv_window)
            self.U_H_Value = cv2.getTrackbarPos("Up_H", self.hsv_window)
            self.U_S_Value = cv2.getTrackbarPos("Up_S", self.hsv_window)
            self.U_V_Value = cv2.getTrackbarPos("Up_V", self.hsv_window)

        # 다양한 HSV 범위를 조정하기 위한 트랙바 생성
        cv2.createTrackbar("Low_H", self.hsv_window, 0, 179, hsv_track)  # H의 최소 임계 값 트랙바 생성
        cv2.createTrackbar("Low_S", self.hsv_window, 0, 255, hsv_track)  # S의 최소 임계 값 트랙바 생성
        cv2.createTrackbar("Low_V", self.hsv_window, 0, 255, hsv_track)  # V의 최소 임계 값 트랙바 생성
        cv2.createTrackbar("Up_H", self.hsv_window, 179, 179, hsv_track)  # H의 최대 임계 값 트랙바 생성
        cv2.createTrackbar("Up_S", self.hsv_window, 255, 255, hsv_track)  # S의 최대 임계 값 트랙바 생성
        cv2.createTrackbar("Up_V", self.hsv_window, 255, 255, hsv_track)  # V의 최대 임계 값 트랙바 생성
        self.create_trackbar_flag = True  # 트랙바가 생성되었음을 표시합니다.

    def apply_mask(self, cv_img):
        # BGR 이미지를 HSV 이미지로 변환합니다.
        cvt_hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # 이미지 블러 처리
        blur = cv2.GaussianBlur(cvt_hsv, (5, 5), 0)

        # HSV 범위를 트랙바의 값으로 정의합니다.
        lower = np.array([self.L_H_Value, self.L_S_Value, self.L_V_Value])  # HSV의 하한 범위를 정의합니다.
        upper = np.array([self.U_H_Value, self.U_S_Value, self.U_V_Value])  # HSV의 상한 범위를 정의합니다.

        # HSV 범위를 사용하여 이미지에 마스크를 생성합니다.
        mask = cv2.inRange(blur, lower, upper)

        # 원본 이미지에 마스크를 적용하여 차선만 있는 부분을 남긴 최종 결과 이미지를 생성합니다.
        hsv_img = cv2.bitwise_and(cv_img, cv_img, mask=mask)
        return hsv_img  # 마스크를 적용한 이미지를 반환합니다.
    
    def binary(self, croped_img):
        # 이진 이미지를 생성하는 메서드입니다. 이진 이미지는 차선을 강조하는 데 사용됩니다.

        bin = cv2.cvtColor(croped_img, cv2.COLOR_BGR2GRAY)  # BGR 이미지를 그레이스케일로 변환합니다.
        binary_img = np.zeros_like(bin)  # 크기가 같은 빈 이진 이미지를 생성합니다.
        binary_img[bin != 0] = 255  # 이진 이미지에서 차선 픽셀을 1로 설정합니다.
        return binary_img  # 이진 이미지를 반환합니다.
    
    def sense(self):
        # 이미지를 "감지(sense)"하는 함수입니다.
        img_msg = self.img_msg  # 수신한 이미지 메세지를 변수에 저장합니다.
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")  # CvBridge 모듈을 사용하여 이미지 메세지(Image)를 OpenCV 이미지로 변환합니다.

        if self.create_trackbar_flag == False:
            self.create_trackbar_init(cv_img)  # 트랙바 초기화 함수를 호출합니다.
        return cv_img  # 처리된 이미지를 반환합니다.

    def think(self, cv_img):
        # 이미지를 분석하고 "생각(think)"하는 함수입니다.
        hsv_img = self.apply_mask(cv_img)  # 이미지에서 마스크를 적용하여 HSV 이미지로 변환합니다.
        binary_img = self.binary(hsv_img)  # 이진 이미지로 변환합니다.

        return hsv_img, binary_img

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
                hsv_img, binary_img= self.think(cv_img)  # 이미지 분석 및 "생각" 함수를 호출합니다.

                cv2.imshow(self.original_window, cv_img)  # 원본 이미지를 OpenCV 창에 표시합니다.
                cv2.imshow(self.hsv_window, hsv_img)  # 처리된 이미지를 OpenCV 창에 표시합니다.
                cv2.imshow(self.bin_window, binary_img)  # 이진화 처리된 이미지를 OpenCV 창에 표시합니다.

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