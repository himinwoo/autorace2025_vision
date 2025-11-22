#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np
from math import pi, atan2

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("image_processor_node", anonymous=True)

        rospy.Subscriber("/usb_cam/image_rect_color", CompressedImage, self.cam_CB)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.original_window = "original_image"
        self.cropped_window = "cropped_image"
        self.binary_window = "bin"
        self.control_window = "control"

        self.create_trackbar_flag = False
        self.cam_flag = False
        self.window_num = 12
        self.margin = 30
        self.previous_point = 0
        self.img_msg = CompressedImage()
        self.cmd_msg = Twist()

        # Trackbar 변수 초기화
        self.L_H_Value = 20
        self.L_S_Value = 100
        self.L_V_Value = 100
        self.U_H_Value = 30
        self.U_S_Value = 255
        self.U_V_Value = 255
        self.L_R_Value = 0
        self.Stop_or_Go = 0
        self.Speed_Value = 1.0  # 속도 값 (m/s)
        self.Steering_Standard = 0

    def create_trackbar_init(self, cv_img):
        cv2.namedWindow(self.original_window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.cropped_window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.binary_window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.control_window)

        def hsv_track(value):
            self.L_H_Value = cv2.getTrackbarPos("Low_H", self.cropped_window)
            self.L_S_Value = cv2.getTrackbarPos("Low_S", self.cropped_window)
            self.L_V_Value = cv2.getTrackbarPos("Low_V", self.cropped_window)
            self.U_H_Value = cv2.getTrackbarPos("Up_H", self.cropped_window)
            self.U_S_Value = cv2.getTrackbarPos("Up_S", self.cropped_window)
            self.U_V_Value = cv2.getTrackbarPos("Up_V", self.cropped_window)

        def control_track(value):
            self.Stop_or_Go = cv2.getTrackbarPos("Stop/Go", self.control_window)
            self.Speed_Value = cv2.getTrackbarPos("Speed", self.control_window)
            self.Steering_Standard = cv2.getTrackbarPos("Steering Standard", self.control_window)

        # Trackbar 생성
        cv2.createTrackbar("Low_H", self.cropped_window, 0, 179, hsv_track)
        cv2.createTrackbar("Low_S", self.cropped_window, 0, 255, hsv_track)
        cv2.createTrackbar("Low_V", self.cropped_window, 0, 255, hsv_track)
        cv2.createTrackbar("Up_H", self.cropped_window, 179, 179, hsv_track)
        cv2.createTrackbar("Up_S", self.cropped_window, 255, 255, hsv_track)
        cv2.createTrackbar("Up_V", self.cropped_window, 255, 255, hsv_track)
        cv2.createTrackbar("Stop/Go", self.control_window, 0, 1, control_track)
        cv2.createTrackbar("Speed", self.control_window, 0, 30, control_track)
        cv2.createTrackbar("Steering Standard", self.control_window, cv_img.shape[1] // 4, cv_img.shape[1] // 2, control_track)

        self.create_trackbar_flag = True

    def birds_eyeview(self, cv_img):
        """Bird's Eye View 변환 함수"""
        src_points = np.float32([
            [0, cv_img.shape[0]],
            [300, 50],
            [cv_img.shape[1] - 300, 50],
            [cv_img.shape[1], cv_img.shape[0]]
        ])
        dst_points = np.float32([
            [0, cv_img.shape[0]],
            [0, 0],
            [cv_img.shape[1], 0],
            [cv_img.shape[1], cv_img.shape[0]]
        ])
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(cv_img, matrix, (cv_img.shape[1], cv_img.shape[0]))
        return warped_img

    def apply_mask(self, warped_img):
        """HSV 마스크와 Canny Edge Detection 적용"""
        hsv_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
        blur = cv2.GaussianBlur(hsv_img, (5, 5), 0)

        lower = np.array([self.L_H_Value, self.L_S_Value, self.L_V_Value])
        upper = np.array([self.U_H_Value, self.U_S_Value, self.U_V_Value])
        mask = cv2.inRange(blur, lower, upper)

        edges = cv2.Canny(mask, 50, 150)  # Canny Edge Detection 적용
        masked_img = cv2.bitwise_and(warped_img, warped_img, mask=mask)

        return masked_img, edges

    def binary(self, croped_img):
        """이진화 처리 및 bin 창 표시"""
        bin_img = cv2.cvtColor(croped_img, cv2.COLOR_BGR2GRAY)
        _, binary_img = cv2.threshold(bin_img, 1, 255, cv2.THRESH_BINARY)
        cv2.imshow(self.binary_window, binary_img)  # bin 창에 표시
        return binary_img

    def sliding_window(self, binary_img):
        """슬라이딩 윈도우 적용 및 중심점 계산"""
        histogram = np.sum(binary_img[binary_img.shape[0] // 2:, :], axis=0)
        midpoint = int(histogram.shape[0] // 2)  # np.int -> int 변경
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint

        avg_point = (left_base + right_base) // 2
        self.previous_point = avg_point

        return avg_point

    def sense(self):
        """이미지 수신 및 초기 설정"""
        cv_img = self.bridge.compressed_imgmsg_to_cv2(self.img_msg)
        if not self.create_trackbar_flag:
            self.create_trackbar_init(cv_img)
        return cv_img

    def think(self, cv_img):
        """이미지 처리 및 분석"""
        warped_img = self.birds_eyeview(cv_img)
        masked_img, _ = self.apply_mask(warped_img)
        binary_img = self.binary(masked_img)
        avg_point = self.sliding_window(binary_img)
        return warped_img, avg_point

    def act(self, warped_img, avg_point):
        """로봇 제어 명령 전송"""
        if self.Stop_or_Go == 0:
            self.cmd_msg.linear.x = 0
            self.cmd_msg.angular.z = 0
        else:
            error = avg_point - self.Steering_Standard
            self.cmd_msg.linear.x = self.Speed_Value / 10
            self.cmd_msg.angular.z = -0.01 * error  # 비례 제어 적용

        self.pub.publish(self.cmd_msg)

    def cam_CB(self, msg):
        """카메라 콜백 함수"""
        self.img_msg = msg
        self.cam_flag = True

    def run(self):
        """메인 루프"""
        if self.cam_flag:
            cv_img = self.sense()
            warped_img, avg_point = self.think(cv_img)
            self.act(warped_img, avg_point)
            cv2.imshow(self.original_window, warped_img)
            cv2.waitKey(1)

def main():
    try:
        processor = ImageProcessor()
        while not rospy.is_shutdown():
            processor.run()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
