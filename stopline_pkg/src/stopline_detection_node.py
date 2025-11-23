#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy 
from coss_msgs.msg import Coss
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from stopline_detector import StopLineDetector

class StopLineDetectionNode:
    """
    정지선 감지 ROS 노드
    카메라 이미지를 구독하고 정지선 감지 결과를 발행합니다.
    """
    def __init__(self):
        rospy.init_node("stopline_detection_node")

        # 파라미터 로드
        self.down_hist_start_line = rospy.get_param('~down_hist_start_line', 0)
        self.stopline_threshold = rospy.get_param('~stopline_threshold', 10)
        self.count_threshold = rospy.get_param('~count_threshold', 10)
        self.cooldown_duration = rospy.get_param('~cooldown_duration', 3.0)
        self.show_debug_image = rospy.get_param('~show_debug_image', False)
        
        # 이미지 크롭 파라미터
        self.crop_top = rospy.get_param('~crop_top', 280)  # 크롭 시작 위치 (위에서부터)
        
        # 히스토그램 임계값 파라미터
        self.histogram_threshold = rospy.get_param('~histogram_threshold', 600)
        
        # Otsu 임계값 조정 파라미터
        self.otsu_threshold_offset = rospy.get_param('~otsu_threshold_offset', 40)
        
        # 허프 변환 파라미터
        self.use_hough = rospy.get_param('~use_hough', True)
        self.hough_threshold = rospy.get_param('~hough_threshold', 120)
        self.hough_min_line_length = rospy.get_param('~hough_min_line_length', 150)
        self.hough_max_line_gap = rospy.get_param('~hough_max_line_gap', 100)
        self.hough_angle_threshold = rospy.get_param('~hough_angle_threshold', 30)
        self.hough_width_ratio = rospy.get_param('~hough_width_ratio', 0.3)

        # 퍼블리셔 설정
        self.coss_pub = rospy.Publisher('/camera/stopline/count', Coss, queue_size=1)
        
        # Coss 메시지 초기화
        self.coss_msg = Coss()
        self.coss_msg.mission_state = 0

        # 서브스크라이버 설정
        rospy.Subscriber("/usb_cam/image_rect_color", Image, self.image_callback)

        # 변수 초기화
        self.bridge = CvBridge()
        self.img = None
        
        # 정지선 감지기 초기화
        self.stopline_detector = StopLineDetector(
            down_hist_start_line=self.down_hist_start_line,
            stopline_threshold=self.stopline_threshold,
            count_threshold=self.count_threshold,
            histogram_threshold=self.histogram_threshold,
            use_hough=self.use_hough,
            hough_threshold=self.hough_threshold,
            hough_min_line_length=self.hough_min_line_length,
            hough_max_line_gap=self.hough_max_line_gap,
            hough_angle_threshold=self.hough_angle_threshold,
            hough_width_ratio=self.hough_width_ratio
        )
        
        # 정지선 감지 쿨다운 설정
        self.last_detection_time = 0.0  # 마지막 정지선 감지 시간
        
        rospy.loginfo("===== Stopline Detection Node Started =====")
        rospy.loginfo(f"Parameters:")
        rospy.loginfo(f"  - crop_top: {self.crop_top}")
        rospy.loginfo(f"  - down_hist_start_line: {self.down_hist_start_line}")
        rospy.loginfo(f"  - stopline_threshold: {self.stopline_threshold}")
        rospy.loginfo(f"  - count_threshold: {self.count_threshold}")
        rospy.loginfo(f"  - cooldown_duration: {self.cooldown_duration}s")
        rospy.loginfo(f"  - show_debug_image: {self.show_debug_image}")
        rospy.loginfo(f"  - histogram_threshold: {self.histogram_threshold}")
        rospy.loginfo(f"  - otsu_threshold_offset: {self.otsu_threshold_offset}")
        rospy.loginfo(f"  - use_hough: {self.use_hough}")
        rospy.loginfo(f"  - hough_threshold: {self.hough_threshold}")
        rospy.loginfo(f"  - hough_min_line_length: {self.hough_min_line_length}")
        rospy.loginfo(f"  - hough_max_line_gap: {self.hough_max_line_gap}")
        rospy.loginfo(f"  - hough_angle_threshold: {self.hough_angle_threshold}")
        rospy.loginfo(f"  - hough_width_ratio: {self.hough_width_ratio}")
        
        # 메인 루프
        rate = rospy.Rate(20)  # 20Hz
        while not rospy.is_shutdown():
            if self.img is not None:
                self.process_image()
            rate.sleep()
    
    def process_image(self):
        """
        이미지를 처리하여 정지선을 감지합니다.
        """
        # 원본 이미지 복사 및 하단 크롭
        stopline_img = self.img[self.crop_top:, :].copy()  # 상단 crop_top 픽셀 제거
        
        # 그레이스케일 변환
        stopline_gray = cv2.cvtColor(stopline_img, cv2.COLOR_BGR2GRAY)
        
        # 블러링으로 노이즈 제거 (빛 반사 완화)
        stopline_gray_blurred = cv2.GaussianBlur(stopline_gray, (5, 5), 0)
        
        # 방법 1: CLAHE + Otsu (조명 변화에 강함)
        clahe = cv2.createCLAHE(clipLimit=1.5, tileGridSize=(8,8))  # clipLimit 낮춤 (2.0 -> 1.5)
        enhanced = clahe.apply(stopline_gray_blurred)
        
        # Otsu 임계값 계산
        otsu_thresh, stopline_bin_otsu = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Otsu 임계값을 조정하여 더 밝은 영역만 선택 (둔감하게)
        adjusted_thresh = otsu_thresh + self.otsu_threshold_offset
        _, stopline_bin_otsu_adjusted = cv2.threshold(enhanced, adjusted_thresh, 255, cv2.THRESH_BINARY)
        
        # 방법 2: 에지 검출 (정지선 경계 강조) - 임계값 높임
        edges = cv2.Canny(stopline_gray_blurred, 100, 200)  # 50,150 -> 100,200 (둔감하게)
        
        # 에지를 두껍게 만들어 히스토그램에서 감지 용이하게
        kernel = np.ones((3, 3), np.uint8)
        edges_dilated = cv2.dilate(edges, kernel, iterations=2)
        
        # Otsu 결과와 에지 결합
        stopline_bin_combined = cv2.bitwise_or(stopline_bin_otsu_adjusted, edges_dilated)
        
        # 0과 1로 정규화
        stopline_bin = (stopline_bin_combined > 0).astype(np.uint8)
        
        # 정지선 감지 수행
        detected, stopline_indices, stopline_count = self.stopline_detector.detect(stopline_bin)
        
        # 정지선 감지 및 쿨다운 처리
        if detected and self.can_detect():
            self.last_detection_time = rospy.get_time()
            self.stopline_detector.reset_count()
            rospy.loginfo(f"정지선 감지! mission_state 업데이트: {self.coss_msg.mission_state} -> {self.coss_msg.mission_state + 1}")
            
            # mission_state 증가
            self.coss_msg.mission_state += 1
        
        # Coss 메시지 발행
        self.coss_pub.publish(self.coss_msg)
        
        # 디버그 이미지 표시 (imshow)
        if self.show_debug_image:
            debug_img = stopline_img.copy()
            self.stopline_detector.draw_stopline(debug_img, stopline_indices)
            
            # 카운트 정보 표시
            cv2.putText(debug_img, f"Count: {stopline_count}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # mission_state 표시
            cv2.putText(debug_img, f"Mission State: {self.coss_msg.mission_state}", (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            
            if detected:
                cv2.putText(debug_img, "STOPLINE DETECTED!", (10, 110), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # 이진화 이미지를 시각화 (0과 1을 0과 255로 변환)
            stopline_bin_visual = (stopline_bin * 255).astype(np.uint8)
            
            # 이미지 출력 (디버깅용 추가)
            cv2.imshow("Stopline Detection", debug_img)
            cv2.imshow("Stopline Binary", stopline_bin_visual)
            cv2.imshow("Stopline CLAHE+Otsu", stopline_bin_otsu_adjusted)
            cv2.imshow("Stopline Edges", edges_dilated)
            cv2.imshow("Stopline Enhanced", enhanced)
            cv2.waitKey(1)
    
    def image_callback(self, msg):
        """
        카메라 이미지 콜백 함수
        ROS 이미지 메시지를 OpenCV 포맷으로 변환
        """
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def can_detect(self):
        """
        정지선 감지 가능 여부를 확인 (쿨다운 체크)
        
        Returns:
            bool: 감지 가능하면 True, 쿨다운 중이면 False
        """
        current_time = rospy.get_time()
        time_since_last_detection = current_time - self.last_detection_time
        
        # 쿨다운 시간이 지났는지 확인
        if time_since_last_detection >= self.cooldown_duration:
            return True
        else:
            remaining_time = self.cooldown_duration - time_since_last_detection
            rospy.logdebug(f"정지선 감지 쿨다운 중... 남은 시간: {remaining_time:.1f}초")
            return False

if __name__ == "__main__":
    try: 
        node = StopLineDetectionNode()
    except rospy.ROSInterruptException:
        pass
