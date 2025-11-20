#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy 
from coss_msgs.msg import Coss
from std_msgs.msg import Float64, Int64, Bool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from math import radians
from slidewindow import SlideWindow
from stopline_detector import StopLineDetector
import cv2
import numpy as np

class PID():
    """
    PID 제어기 클래스: 차선 추종을 위한 제어 알고리즘
    """
    def __init__(self, kp, ki, kd):
        self.kp = kp  # 비례 게인
        self.ki = ki  # 적분 게인
        self.kd = kd  # 미분 게인
        self.p_error = 0.0  # 이전 오차
        self.i_error = 0.0  # 누적 오차
        self.d_error = 0.0  # 오차의 변화율

    def pid_control(self, cte):
        """
        현재 오차(cte)를 기반으로 제어값 계산
        cte: 현재 차량과 목표 위치 사이의 차이(Cross Track Error)
        """
        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte
        return self.kp * self.p_error + self.ki * self.i_error + self.kd * self.d_error

class LaneDetection:
    def __init__(self):
        rospy.init_node("lane_detection_node")

        # 퍼블리셔 설정
        self.camera_pub = rospy.Publisher('/camera', Coss, queue_size=1)

        # 서브스크라이버 설정
        rospy.Subscriber("/usb_cam/image_rect_color", Image, self.camCB)

        # 변수 초기화
        self.bridge = CvBridge()
        
        self.camera_msg = Coss()
        self.camera_msg.cam_steer = 0.0
        self.camera_msg.mission_state = 0

        # 슬라이딩 윈도우 객체 초기화
        self.slidewindow = SlideWindow()  # 첫 번째 구간용
        
        # 정지선 감지기 초기화
        self.stopline_detector = StopLineDetector(
            down_hist_start_line=400,
            stopline_threshold=10,
            count_threshold=10
        )

        # 차선 감지 관련 변수
        self.stopline_count = 0
        self.img = []  # 카메라 이미지
        self.warped_img = []  # 원근 변환 이미지
        self.grayed_img = []  # 그레이스케일 이미지
        self.bin_img = []  # 이진화 이미지
        self.out_img = []  # 출력 이미지
        
        # 정지선 감지 쿨다운 설정
        self.state_cooldown_duration = 3.0  # 3초간 중복 감지 방지
        self.last_state_update_time = 0.0  # 마지막 상태 업데이트 시간
        
        # 차선 위치 추적용 변수
        self.x_location = 320  # 현재 차선 중심 x 좌표 (640 → 320)
        self.last_x_location = 320  # 이전 차선 중심 x 좌표 (640 → 320)
        self.center_index = 320  # 추적 중심점 (640 → 320)
        
        # 히스토그램 영역 설정
        self.up_hist_end_line = 400  # 상단 히스토그램 끝 라인

        rate = rospy.Rate(20)  # 20Hz로 실행
        while not rospy.is_shutdown():
            if len(self.img) != 0:
                self.process_image()
            rate.sleep()
    
    def process_image(self):
        """
        이미지를 처리하여 차선을 감지하고 제어 명령을 계산
        """
        y, x = self.img.shape[0:2]
        
        # 원본 이미지 복사 (정지선 감지용)
        stopline_img = self.img.copy()
        
        # ROI 마스크 생성 - 위쪽 50%를 검게 만들기 (차선 감지용)
        roi_mask = np.zeros((y, x), dtype=np.uint8)
        crop_height = y // 2
        roi_mask[crop_height:, :] = 255  # 아래쪽 50%만 흰색으로
        
        # 원본 이미지에 ROI 마스크 적용 (차선 감지용)
        masked_img = cv2.bitwise_and(self.img, self.img, mask=roi_mask)
        
        # 1. 이미지 전처리 - HSV 변환 및 색상 필터링
        self.img_hsv = cv2.cvtColor(masked_img, cv2.COLOR_BGR2HSV)
        
        # 노란색 차선 감지
        yellow_lower = np.array([10, 50, 50])
        yellow_upper = np.array([40, 255, 255])
        self.yellow_range = cv2.inRange(self.img_hsv, yellow_lower, yellow_upper)
        
        # # 흰색 차선 감지 (두 가지 범위 사용)
        # white_lower_bound1 = np.array([0, 0, 130])
        # white_upper_bound1 = np.array([179, 30, 255])
        # self.white_range = cv2.inRange(self.img_hsv, white_lower_bound1, white_upper_bound1)
                
        # 노란색과 흰색 마스크 통합
        # combined_range = cv2.bitwise_or(self.yellow_range, self.white_range)
        combined_range = self.yellow_range
        
        # 가우시안 블러로 노이즈 제거
        combined_range = cv2.GaussianBlur(combined_range, (5, 5), 0)
        
        # 모폴로지 연산 - 노이즈 제거 및 차선 연결
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        combined_range = cv2.morphologyEx(combined_range, cv2.MORPH_CLOSE, kernel)  # 작은 구멍 메우기
        combined_range = cv2.morphologyEx(combined_range, cv2.MORPH_OPEN, kernel)   # 작은 노이즈 제거
        
        result = combined_range
        filtered_img = cv2.bitwise_and(masked_img, masked_img, mask=result)
        
        # 2. 원근 변환 - 버드아이 뷰로 변환
        # 소스 포인트 설정 (원본 이미지에서의 사다리꼴 영역)
        # 640x480 해상도에 맞게 조정
        S_Upper_X = 171
        S_Upper_Y = 240
        S_Lower_X = 0
        S_Lower_Y = 58
        
        src_point1 = [0 + S_Lower_X, 480 - S_Lower_Y]      # 왼쪽 아래
        src_point2 = [0 + S_Upper_X, 0 + S_Upper_Y]        # 왼쪽 위
        src_point3 = [640 - S_Upper_X, 0 + S_Upper_Y]      # 오른쪽 위 (640 - 180 = 460)
        src_point4 = [640 - S_Lower_X, 480 - S_Lower_Y]    # 오른쪽 아래
        
        src_points = np.float32([src_point1, src_point2, src_point3, src_point4])
        
        # 목적지 포인트 설정 (변환된 이미지에서의 직사각형 영역)
        # 640x480 해상도에 맞게 조정
        D_Upper_X = 182
        D_Upper_Y = 88
        D_Lower_X = 207
        D_Lower_Y = 0
        
        dst_point1 = [0 + D_Lower_X, 480 - D_Lower_Y]      # 왼쪽 아래 (165, 480)
        dst_point2 = [0 + D_Upper_X, 0 + D_Upper_Y]        # 왼쪽 위 (125, 0)
        dst_point3 = [640 - D_Upper_X, 0 + D_Upper_Y]      # 오른쪽 위 (515, 0)
        dst_point4 = [640 - D_Lower_X, 480 - D_Lower_Y]    # 오른쪽 아래 (475, 480)
        
        dst_points = np.float32([dst_point1, dst_point2, dst_point3, dst_point4])
        
        # 변환 행렬 계산 및 적용
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        self.warped_img = cv2.warpPerspective(filtered_img, matrix, (x, y))
        self.grayed_img = cv2.cvtColor(self.warped_img, cv2.COLOR_BGR2GRAY)
        
        # 3. 이미지 이진화 - 차선 픽셀 추출
        self.bin_img = np.zeros_like(self.grayed_img)
        self.bin_img[self.grayed_img > 60] = 1  # 밝기가 60 이상인 픽셀만 선택
        
        # 4. 정지선 감지 (원본 이미지 별도 처리)
        # 정지선 감지용 이미지 전처리
        stopline_hsv = cv2.cvtColor(stopline_img, cv2.COLOR_BGR2HSV)
        
        # 흰색 정지선 감지
        white_lower = np.array([0, 0, 130])
        white_upper = np.array([179, 30, 255])
        white_mask = cv2.inRange(stopline_hsv, white_lower, white_upper)
        
        # 정지선 감지용 이미지 이진화
        stopline_gray = cv2.cvtColor(stopline_img, cv2.COLOR_BGR2GRAY)
        stopline_bin = np.zeros_like(stopline_gray)
        stopline_bin[white_mask > 0] = 1
        
        # 정지선 감지 수행
        detected, stopline_indices, self.stopline_count = self.stopline_detector.detect(stopline_bin)
                
        # 정지선 감지 및 쿨다운 처리
        if detected and self.can_update_state():
            self.camera_msg.mission_state += 1  # 정지선 감지 상태 설정
            self.last_state_update_time = rospy.get_time()  # 현재 시간 기록
            self.stopline_detector.reset_count()  # 카운터 리셋하여 중복 감지 방지
            rospy.loginfo(f"정지선 감지! 미션 상태: {self.camera_msg.mission_state}")


        # 5. 슬라이딩 윈도우로 차선 검출
        # 이전 위치 저장 (차선 추적 실패 시 사용)
        if self.x_location is not None:
            self.last_x_location = self.x_location
        
        # 슬라이딩 윈도우 알고리즘 적용
        # cam_mission_state에 따라 차선 감지 모드 결정
        # 0: 양쪽 차선, 1: 왼쪽만, 2: 오른쪽만
        self.out_img, self.x_location, _ = self.slidewindow.slidewindow(
            self.bin_img, 
            lane_mode=0
        )
        pid = PID(0.020, 0.01, 0.010)  # PID 파라미터
        
        # 정지선 표시 (디버깅용)
        self.stopline_detector.draw_stopline(self.out_img, stopline_indices)
        
        # 차선 검출 실패 시 이전 위치 사용
        if self.x_location is None:
            self.x_location = self.last_x_location
        
        # 6. PID 제어 및 조향각 계산
        self.center_index = self.x_location
        angle = pid.pid_control(self.center_index - 320)  # 이미지 중앙(320)과의 오차 계산
        
        # 7. 제어 명령 발행 
        self.camera_msg.cam_steer = -radians(angle)  # 각도를 라디안으로 변환 (조향각)

        # 통합 메시지 발행
        self.camera_pub.publish(self.camera_msg)

        
        # 결과 이미지 표시
        cv2.imshow("out_img", self.out_img)
        cv2.imshow("combined_range", combined_range)
        cv2.waitKey(1)

    def camCB(self, msg):
        """
        카메라 이미지 콜백 함수
        ROS 이미지 메시지를 opencv 포맷으로 변환
        """
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def can_update_state(self):
        """
        상태 업데이트 가능 여부를 확인 (쿨다운 체크)
        Returns:
            bool: 상태 업데이트 가능하면 True, 쿨다운 중이면 False
        """
        current_time = rospy.get_time()
        time_since_last_update = current_time - self.last_state_update_time
        
        # 쿨다운 시간이 지났는지 확인
        if time_since_last_update >= self.state_cooldown_duration:
            return True
        else:
            # 디버깅용 로그 (선택적)
            remaining_time = self.state_cooldown_duration - time_since_last_update
            rospy.logdebug(f"정지선 감지 쿨다운 중... 남은 시간: {remaining_time:.1f}초")
            return False

if __name__ == "__main__":
    try: 
        lane_detection_node = LaneDetection()
    except rospy.ROSInterruptException:
        pass