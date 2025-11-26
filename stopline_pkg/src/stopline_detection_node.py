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
        
        # 로그 레벨 설정
        log_level = rospy.get_param('~log_level', 'info')
        if log_level.lower() == 'debug':
            rospy.loginfo("Setting log level to DEBUG")
            import logging
            logger = logging.getLogger('rosout')
            logger.setLevel(logging.DEBUG)

        # 파라미터 로드
        self.down_hist_start_line = rospy.get_param('~down_hist_start_line', 0)
        self.stopline_threshold = rospy.get_param('~stopline_threshold', 10)
        self.count_threshold = rospy.get_param('~count_threshold', 15)
        self.cooldown_durations = rospy.get_param('~cooldown_durations', [6.0])  # mission_state별 쿨다운 시간 리스트
        self.show_debug_image = rospy.get_param('~show_debug_image', False)
        
        # 이미지 크롭 파라미터
        self.crop_top = rospy.get_param('~crop_top', 280)  # 크롭 시작 위치 (위에서부터)
        
        # 히스토그램 임계값 파라미터
        self.histogram_threshold = rospy.get_param('~histogram_threshold', 800)
        
        # Otsu 임계값 조정 파라미터
        self.otsu_threshold_offset = rospy.get_param('~otsu_threshold_offset', 50)
        
        # 허프라인 파라미터
        self.hough_threshold = rospy.get_param('~hough_threshold', 50)
        self.hough_min_line_length = rospy.get_param('~hough_min_line_length', 40)
        self.hough_max_line_gap = rospy.get_param('~hough_max_line_gap', 10)
        self.hough_angle_tolerance = rospy.get_param('~hough_angle_tolerance', 25.0)  # 수평선 기준 ±각도
        self.hough_use_morphology = rospy.get_param('~hough_use_morphology', True)  # 허프라인 결과에 모폴로지 적용
        
        # 색공간 선택 파라미터
        self.use_lab_colorspace = rospy.get_param('~use_lab_colorspace', True)  # LAB 색공간 사용 여부 (빛 번짐에 강함)
        
        # 노란색 정지선 감지 파라미터 (state 9용)
        self.yellow_hsv_lower = rospy.get_param('~yellow_hsv_lower', [0, 40, 50])
        self.yellow_hsv_upper = rospy.get_param('~yellow_hsv_upper', [26, 110, 255])
        self.yellow_state = rospy.get_param('~yellow_state', 9)  # 노란색 감지할 state

        # 퍼블리셔 설정
        self.coss_pub = rospy.Publisher('/camera/stopline/count', Coss, queue_size=1)
        
        # Coss 메시지 초기화
        self.coss_msg = Coss()
        self.coss_msg.mission_state = 0
        
        # Lidar flag 관련 변수
        self.lidar_flag = False
        self.lidar_cooldown_started = False  # state 6에서 쿨다운 시작 여부
        
        # Cone finish 관련 변수 (state 3, 7에서 정지선 감지 제어)
        self.cone_finish = False

        # 서브스크라이버 설정
        rospy.Subscriber("/usb_cam/image_rect_color", Image, self.image_callback)
        rospy.Subscriber("/lidar", Coss, self.lidar_callback)  # Coss 메시지 구독 (lidar_flag, cone_finish)

        # 변수 초기화
        self.bridge = CvBridge()
        self.img = None
        
        # 정지선 감지기 초기화
        self.stopline_detector = StopLineDetector(
            down_hist_start_line=self.down_hist_start_line,
            stopline_threshold=self.stopline_threshold,
            count_threshold=self.count_threshold,
            histogram_threshold=self.histogram_threshold,
            otsu_threshold_offset=self.otsu_threshold_offset
        )
        
        # 정지선 감지 쿨다운 설정
        self.last_detection_time = 0.0  # 마지막 정지선 감지 시간 (또는 쿨다운 시작 시간)
        
        rospy.loginfo("===== Stopline Detection Node Started =====")
        rospy.loginfo(f"Parameters:")
        rospy.loginfo(f"  - crop_top: {self.crop_top}")
        rospy.loginfo(f"  - down_hist_start_line: {self.down_hist_start_line}")
        rospy.loginfo(f"  - stopline_threshold: {self.stopline_threshold}")
        rospy.loginfo(f"  - count_threshold: {self.count_threshold}")
        rospy.loginfo(f"  - cooldown_durations: {self.cooldown_durations}")
        rospy.loginfo(f"  - show_debug_image: {self.show_debug_image}")
        rospy.loginfo(f"  - histogram_threshold: {self.histogram_threshold}")
        rospy.loginfo(f"  - otsu_threshold_offset: {self.otsu_threshold_offset}")
        rospy.loginfo(f"  - hough_threshold: {self.hough_threshold}")
        rospy.loginfo(f"  - hough_min_line_length: {self.hough_min_line_length}")
        rospy.loginfo(f"  - hough_max_line_gap: {self.hough_max_line_gap}")
        rospy.loginfo(f"  - hough_angle_tolerance: {self.hough_angle_tolerance}")
        rospy.loginfo(f"  - hough_use_morphology: {self.hough_use_morphology}")
        rospy.loginfo(f"  - use_lab_colorspace: {self.use_lab_colorspace}")
        rospy.loginfo(f"  - yellow_hsv_lower: {self.yellow_hsv_lower}")
        rospy.loginfo(f"  - yellow_hsv_upper: {self.yellow_hsv_upper}")
        rospy.loginfo(f"  - yellow_state: {self.yellow_state}")
        
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
        
        # state 9일 때는 노란색 필터링 적용
        if self.coss_msg.mission_state == self.yellow_state:
            stopline_bin = self.detect_yellow_stopline(stopline_img)
        else:
            stopline_bin = self.detect_white_stopline(stopline_img)
        
        # 정지선 감지 수행
        detected, stopline_indices, stopline_count = self.stopline_detector.detect(stopline_bin)
        
        # 정지선 감지 및 쿨다운 처리
        if detected and self.can_detect():
            self.last_detection_time = rospy.get_time()
            self.stopline_detector.reset_count()
            rospy.loginfo(f"정지선 감지! mission_state 업데이트: {self.coss_msg.mission_state} -> {self.coss_msg.mission_state + 1}")
            
            # mission_state 증가
            self.coss_msg.mission_state += 1
            
            # state 3, 7 진입 시 cone_finish 초기화 (정지선 감지로 state 변경 시)
            if self.coss_msg.mission_state in [3, 7]:
                self.cone_finish = False
                rospy.loginfo(f"[State {self.coss_msg.mission_state} 진입] cone_finish 초기화")
        
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
            state_text = f"Mission State: {self.coss_msg.mission_state}"
            if self.coss_msg.mission_state == self.yellow_state:
                state_text += " (YELLOW)"
            cv2.putText(debug_img, state_text, (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            
            if detected:
                cv2.putText(debug_img, "STOPLINE DETECTED!", (10, 110), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # 이진화 이미지를 시각화 (0과 1을 0과 255로 변환)
            stopline_bin_visual = (stopline_bin * 255).astype(np.uint8)
            
            # 이미지 출력 (디버깅용 추가)
            cv2.imshow("Stopline Detection", debug_img)
            cv2.imshow("Stopline Binary", stopline_bin_visual)
            cv2.waitKey(1)
    
    def detect_yellow_stopline(self, img):
        """
        노란색 정지선 감지 (state 9용)
        
        Args:
            img: BGR 이미지
            
        Returns:
            stopline_bin: 이진화된 정지선 이미지 (0과 1)
        """
        # HSV 변환
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # 노란색 마스크 생성
        lower_yellow = np.array(self.yellow_hsv_lower, dtype=np.uint8)
        upper_yellow = np.array(self.yellow_hsv_upper, dtype=np.uint8)
        yellow_mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
        
        # 가우시안 블러로 노이즈 제거
        yellow_mask_blurred = cv2.GaussianBlur(yellow_mask, (5, 5), 0)
        
        # 모폴로지 연산으로 노이즈 제거 및 영역 보완
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask_clean = cv2.morphologyEx(yellow_mask_blurred, cv2.MORPH_CLOSE, kernel)
        yellow_mask_clean = cv2.morphologyEx(yellow_mask_clean, cv2.MORPH_OPEN, kernel)
        
        # 0과 1로 정규화
        stopline_bin = (yellow_mask_clean > 0).astype(np.uint8)
        
        return stopline_bin
    
    def detect_white_stopline(self, img):
        """
        흰색 정지선 감지 (히스토그램+허프라인 OR)
        Args:
            img: BGR 이미지
        Returns:
            stopline_bin: 이진화된 정지선 이미지 (0과 1)
        """
        # 색공간 변환: LAB L 채널 또는 그레이스케일
        if self.use_lab_colorspace:
            # LAB 색공간으로 변환 (빛 번짐과 반사에 강함)
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            stopline_gray = lab[:, :, 0]  # L (Lightness) 채널만 추출
        else:
            # 기존 그레이스케일 방식
            stopline_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # ===== 허프라인 검출 (독립적으로 먼저 수행) =====
        hough_mask = self._detect_hough_lines(stopline_gray)
        
        # ===== 히스토그램 기반 검출 =====
        # 1단계: 과포화 영역 마스킹 (빛 반사 사전 제거)
        saturation_mask = (stopline_gray < 230) & (stopline_gray > 30)
        stopline_gray_filtered = cv2.bitwise_and(
            stopline_gray, stopline_gray,
            mask=saturation_mask.astype(np.uint8)
        )
        # 2단계: 적응적 노이즈 제거
        stopline_denoised = cv2.bilateralFilter(stopline_gray_filtered, 9, 75, 75)
        # 3단계: CLAHE + 적응형 임계값 결합
        clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(8,8))
        enhanced = clahe.apply(stopline_denoised)
        # Otsu + offset
        otsu_thresh, _ = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        adjusted_thresh = otsu_thresh + self.otsu_threshold_offset
        _, stopline_bin_otsu = cv2.threshold(enhanced, adjusted_thresh, 255, cv2.THRESH_BINARY)
        # Adaptive threshold (지역적 조명 변화 대응)
        adaptive_thresh = cv2.adaptiveThreshold(
            enhanced, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, blockSize=21, C=-3
        )
        # 두 임계값의 교집합 (더 보수적)
        combined_thresh = cv2.bitwise_and(stopline_bin_otsu, adaptive_thresh)
        # 4단계: 에지 강조 (정지선 경계선)
        edges = cv2.Canny(stopline_denoised, 100, 200)
        kernel = np.ones((3, 3), np.uint8)
        edges_dilated = cv2.dilate(edges, kernel, iterations=1)
        # 임계값과 에지 결합
        stopline_bin_combined = cv2.bitwise_or(combined_thresh, edges_dilated)
        # 5단계: 형태학적 필터링 (원형 객체 제거)
        horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 1))
        horizontal_emphasis = cv2.morphologyEx(
            stopline_bin_combined, cv2.MORPH_OPEN, horizontal_kernel
        )
        kernel = np.ones((5, 5), np.uint8)
        stopline_clean = cv2.morphologyEx(horizontal_emphasis, cv2.MORPH_CLOSE, kernel)
        # 6단계: 원형도 기반 필터링
        histo_bin = self.stopline_detector._filter_circular_regions(stopline_clean)

        # ===== OR 연산으로 결과 합치기 =====
        stopline_bin = cv2.bitwise_or(histo_bin, hough_mask)
        return stopline_bin
    
    def _detect_hough_lines(self, gray_img):
        """
        허프라인 변환을 이용한 정지선 검출
        원본 그레이스케일(또는 LAB L) 이미지에서 직접 검출하여 전처리 손실 최소화
        
        Args:
            gray_img: 그레이스케일 또는 LAB L 채널 이미지
            
        Returns:
            hough_mask: 허프라인 검출 결과 마스크 (0과 1)
        """
        # 1단계: 가우시안 블러로 노이즈 감소 (과도한 전처리 방지)
        blurred = cv2.GaussianBlur(gray_img, (5, 5), 0)
        
        # 2단계: Canny 에지 검출 (원본에 가까운 이미지에서)
        edges = cv2.Canny(blurred, 50, 150, apertureSize=3)
        
        # 3단계: 허프라인 변환으로 직선 검출
        lines = cv2.HoughLinesP(
            edges, 
            rho=1, 
            theta=np.pi/180, 
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length, 
            maxLineGap=self.hough_max_line_gap
        )
        
        # 4단계: 정지선 특성에 맞는 라인 필터링
        hough_mask = np.zeros_like(gray_img)
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # 라인 각도 계산 (수평선 기준)
                angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
                
                # 각도 정규화 (-180 ~ 180)
                if angle < 0:
                    angle += 180
                
                # 수평선에 가까운 라인만 선택 (0도, 180도 근처)
                # 또는 약간의 대각선 (hough_angle_tolerance 이내)
                is_near_horizontal = (
                    angle <= self.hough_angle_tolerance or 
                    angle >= (180 - self.hough_angle_tolerance)
                )
                
                # 정지선 특성: 수평 또는 약간 기울어진 라인
                if is_near_horizontal:
                    # 라인 길이 계산
                    line_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    
                    # 충분히 긴 라인만 사용 (정지선은 이미지 폭의 상당 부분 차지)
                    if line_length > gray_img.shape[1] * 0.15:  # 이미지 폭의 15% 이상
                        # 두꺼운 선으로 그려서 히스토그램과 결합 시 효과적
                        cv2.line(hough_mask, (x1, y1), (x2, y2), 255, thickness=5)
        
        # 5단계: 모폴로지 연산으로 라인 보강 (선택적)
        if self.hough_use_morphology and np.any(hough_mask > 0):
            # 수평 방향 확장 (정지선 연속성 보장)
            horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 3))
            hough_mask = cv2.morphologyEx(hough_mask, cv2.MORPH_CLOSE, horizontal_kernel)
            
            # 수직 방향 확장 (정지선 두께 보장)
            vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 7))
            hough_mask = cv2.dilate(hough_mask, vertical_kernel, iterations=1)
        
        # 0과 1로 정규화
        hough_mask = (hough_mask > 0).astype(np.uint8)
        
        return hough_mask
    
    def image_callback(self, msg):
        """
        카메라 이미지 콜백 함수
        ROS 이미지 메시지를 OpenCV 포맷으로 변환
        """
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def lidar_callback(self, msg):
        """
        /lidar 토픽 콜백 함수
        Coss 메시지에서 lidar_flag와 cone_finish를 저장합니다.
        state 6에서 lidar_flag가 false→true로 전환되면 쿨다운을 시작합니다.
        """
        prev_flag = self.lidar_flag
        self.lidar_flag = msg.lidar_flag
        
        # cone_finish 값 업데이트
        self.cone_finish = msg.cone_finish
        
        # start_parking 값 업데이트
        self.start_parking = msg.start_parking
        
        if self.start_parking == True:
            self.coss_msg.mission_state = 10
        
        # state 6에서만 lidar_flag 전환 감지
        if self.coss_msg.mission_state == 6:
            self._handle_state6_lidar_transition(prev_flag)
        
        # state 3, 7에서 cone_finish가 true로 변경되면 로그 출력
        if self.coss_msg.mission_state in [3, 7] and self.cone_finish:
            rospy.loginfo_once(f"[State {self.coss_msg.mission_state}] cone_finish = True → 정지선 감지 활성화")
        
        rospy.logdebug(f"Received lidar_flag: {self.lidar_flag}, cone_finish: {self.cone_finish}")
    
    def _handle_state6_lidar_transition(self, prev_flag):
        """
        state 6에서 lidar_flag 전환 처리
        false→true 전환 시 쿨다운 타이머를 시작합니다.
        Args:
            prev_flag: 이전 lidar_flag 값
        """
        # 이미 쿨다운이 시작되었으면 무시
        if self.lidar_cooldown_started:
            return
        
        # false에서 true로 전환되는 순간 감지
        if self.lidar_flag and not prev_flag:
            self.lidar_cooldown_started = True
            self.last_detection_time = rospy.get_time()
            rospy.loginfo("[State 6] 라이다 감지 → 쿨다운 시작")
    
    def can_detect(self):
        """
        정지선 감지 가능 여부를 확인 (쿨다운 체크 및 cone_finish 체크)
        
        state 3, 7: cone_finish가 true가 될 때까지 정지선 감지 비활성화
        state 6: 라이다 감지 후 쿨다운이 시작되어야 감지 가능
        기타 state: 이전 감지 시점부터 쿨다운 경과 후 감지 가능
        
        Returns:
            bool: 감지 가능하면 True, 불가능하면 False
        """
        current_state = self.coss_msg.mission_state
        
        # state 3, 7: cone_finish가 true가 아니면 정지선 감지 불가
        if current_state in [3, 7]:
            if not self.cone_finish:
                rospy.logdebug_throttle(1.0, f"[State {current_state}] cone_finish 대기 중...")
                return False
        
        # state 6 특수 처리: 라이다 쿨다운이 시작되지 않았으면 감지 불가
        if current_state == 6 and not self.lidar_cooldown_started:
            rospy.logdebug("[State 6] 라이다 감지 대기 중...")
            return False
        
        # state 10 에서는 정지선 감지하지 않음
        if current_state == 10:
            return False
        
        # 쿨다운 시간 확인
        return self._is_cooldown_complete(current_state)
    
    def _is_cooldown_complete(self, current_state):
        """
        쿨다운이 완료되었는지 확인
        
        Args:
            current_state: 현재 mission_state
            
        Returns:
            bool: 쿨다운이 완료되었으면 True, 아니면 False
        """
        current_time = rospy.get_time()
        elapsed_time = current_time - self.last_detection_time
        
        # 현재 state에 해당하는 쿨다운 시간 가져오기
        cooldown_duration = self._get_cooldown_duration(current_state)
        
        # 쿨다운 완료 여부 확인
        if elapsed_time >= cooldown_duration:
            return True
        else:
            remaining_time = cooldown_duration - elapsed_time
            rospy.logdebug(
                f"[State {current_state}] 쿨다운 진행 중... "
                f"남은 시간: {remaining_time:.1f}초"
            )
            return False
    
    def _get_cooldown_duration(self, state):
        """
        주어진 state에 해당하는 쿨다운 시간을 반환
        
        Args:
            state: mission_state 값
            
        Returns:
            float: 쿨다운 시간(초)
        """
        if state < len(self.cooldown_durations):
            return self.cooldown_durations[state]
        else:
            # state가 리스트 범위를 벗어나면 마지막 값 사용
            return self.cooldown_durations[-1]

if __name__ == "__main__":
    try: 
        node = StopLineDetectionNode()
    except rospy.ROSInterruptException:
        pass
