#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
차선 검출 및 조향 제어 노드

카메라 이미지에서 왼쪽 또는 오른쪽 차선을 검출하고,
Sliding Window 알고리즘을 사용하여 차선의 위치를 추적한 후
Ackermann 조향 명령을 생성하여 발행합니다.

모든 파라미터는 ROS 파라미터 서버를 통해 설정 가능합니다.
"""

import rospy
from sensor_msgs.msg import Image
from coss_msgs.msg import Coss
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from math import pi, radians


class ImageProcessor:
    """차선 검출 및 조향 제어를 담당하는 클래스
    
    ROS 파라미터로 왼쪽/오른쪽 차선 선택 및 HSV 임계값, 조향 게인 등을 설정할 수 있습니다.
    """

    def __init__(self):
        """노드 초기화 및 ROS 파라미터 로드"""
        rospy.init_node("onelane_detection_node", anonymous=True)
        self.bridge = CvBridge()  # ROS Image 메시지를 OpenCV 이미지로 변환하는 브릿지

        # 카메라 이미지 토픽 구독 설정
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_rect_color")
        rospy.Subscriber(self.image_topic, Image, self.cam_CB)

        # 차선 선택 파라미터 ("left" 또는 "right")
        lane_side_param = rospy.get_param("~lane_side", "right").lower()
        if lane_side_param not in ("left", "right"):
            rospy.logwarn("lane_side must be 'left' or 'right'. Falling back to 'left'.")
            lane_side_param = "left"
        self.use_left_lane = lane_side_param == "left"

        # 카메라 제어 명령 발행 토픽 설정
        self.cmd_topic = rospy.get_param("~cmd_topic", "/camera")
        self.pub = rospy.Publisher(self.cmd_topic, Coss, queue_size=1)

        # Sliding Window 알고리즘 파라미터
        self.window_num = int(rospy.get_param("~window_num", 12))  # 윈도우 개수
        self.margin = int(rospy.get_param("~window_margin", 30))  # 윈도우 좌우 마진 (픽셀)
        self.crop_start_ratio = float(rospy.get_param("~crop_start_ratio", 0.5))  # 이미지 상단 자르기 비율 (0.0~1.0)
        self.crop_start_ratio = min(max(self.crop_start_ratio, 0.0), 0.95)

        # HSV 색상 필터링 임계값 (차선 색상 검출용)
        self.hsv_lower = np.array(rospy.get_param("~hsv_lower", [10, 50, 50]), dtype=np.uint8)
        self.hsv_upper = np.array(rospy.get_param("~hsv_upper", [40, 255, 255]), dtype=np.uint8)

        # 조향 기준점 비율 (crop된 이미지 너비에 대한 비율)
        ratio_default = float(rospy.get_param("~steering_ratio", 0.8))
        self.left_ratio = float(rospy.get_param("~steering_ratio_left", ratio_default))
        self.right_ratio = float(rospy.get_param("~steering_ratio_right", ratio_default))

        # 조향 게인 (왼쪽/오른쪽 차선별로 다르게 설정 가능)
        self.left_gain = float(rospy.get_param("~steering_gain_left", 0.66))
        self.right_gain = float(rospy.get_param("~steering_gain_right", 0.55))
        self.gain_override = rospy.get_param("~steering_gain", None)  # 통합 게인 (설정 시 좌우 구분 무시)

        # 주행 제어 파라미터
        self.enable_drive_default = bool(rospy.get_param("~enable_drive", True))  # 주행 활성화 여부
        self.show_debug = bool(rospy.get_param("~show_debug", True))  # 디버그 창 표시 여부

        # 내부 상태 변수
        self.previous_point = 0  # 이전 프레임의 차선 중심점 (검출 실패 시 사용)
        self.Steering_Standard = 0  # 조향 기준점 (픽셀 좌표)
        self.img_msg = None  # 수신한 카메라 이미지 메시지
        self.cam_flag = False  # 카메라 이미지 수신 여부
        self.cmd_msg = Coss()  # 발행할 제어 명령 메시지
        self.cmd_msg.cam_steer = 0.0

    def birds_eyeview(self, cv_img):
        """원근 변환(Perspective Transform)을 적용하여 Bird's Eye View 생성
        
        Args:
            cv_img: 입력 이미지 (원본 카메라 이미지)
            
        Returns:
            warped_img: 원근 변환된 이미지 (위에서 내려다본 시점)
        """
        # 원본 이미지에서 변환할 4개의 점 (사다리꼴 형태)
        # 순서: 좌상단 → 좌하단 → 우하단 → 우상단
        src_points = np.float32([[90, 361 - 35], [0, 361], [640, 361], [640 - 90, 361 - 35]])
        # 변환 후 목적지 점 (직사각형 형태로 펼침)
        # src_points와 동일한 순서: 좌상단 → 좌하단 → 우하단 → 우상단
        dst_points = np.float32([[0, 0], [0, cv_img.shape[0]], [cv_img.shape[1], cv_img.shape[0]], [cv_img.shape[1], 0]])
        # 변환 행렬 계산
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        # 원근 변환 적용
        warped_img = cv2.warpPerspective(cv_img, matrix, (cv_img.shape[1], cv_img.shape[0]))
        return warped_img

    def apply_mask(self, warped_img):
        """HSV 색상 공간에서 차선 색상만 추출
        
        Args:
            warped_img: 원근 변환된 이미지
            
        Returns:
            hsv_img: 차선 색상만 추출된 이미지
        """
        # BGR을 HSV 색상 공간으로 변환
        cvt_hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
        # 가우시안 블러로 노이즈 제거
        blur = cv2.GaussianBlur(cvt_hsv, (5, 5), 0)
        # HSV 임계값 범위 내의 픽셀만 마스크로 선택
        mask = cv2.inRange(blur, self.hsv_lower, self.hsv_upper)
        # 원본 이미지에 마스크를 적용하여 차선만 추출
        hsv_img = cv2.bitwise_and(warped_img, warped_img, mask=mask)
        return hsv_img

    def crop_img(self, hsv_img):
        """관심 영역(ROI)만 추출 - 상단 자르기 + 왼쪽 또는 오른쪽 절반 선택
        
        Args:
            hsv_img: HSV 필터링된 이미지
            
        Returns:
            croped_img: 관심 영역만 잘라낸 이미지
            croped_img_shape: 잘라낸 이미지의 (높이, 너비)
        """
        # 이미지 상단 영역을 crop_start_ratio 비율만큼 제거 (먼 거리 제외)
        start_row = int(hsv_img.shape[0] * self.crop_start_ratio)
        start_row = min(start_row, hsv_img.shape[0] - 1)
        cropped_half = hsv_img[start_row:, :]
        
        # 이미지를 좌우로 이등분
        half_width = cropped_half.shape[1] // 2
        if self.use_left_lane:
            croped_img = cropped_half[:, :half_width]  # 왼쪽 절반 선택
        else:
            croped_img = cropped_half[:, half_width:]  # 오른쪽 절반 선택
        
        croped_img_shape = croped_img.shape[0:2]
        return croped_img, croped_img_shape

    def binary(self, croped_img):
        """이진화 이미지 생성 (차선 픽셀 = 255, 배경 = 0)
        
        Args:
            croped_img: 관심 영역으로 잘라낸 이미지
            
        Returns:
            binary_img: 이진화된 이미지
        """
        # BGR을 그레이스케일로 변환
        gray = cv2.cvtColor(croped_img, cv2.COLOR_BGR2GRAY)
        # 0이 아닌 모든 픽셀을 255로 설정 (차선 픽셀만 흰색으로)
        binary_img = np.zeros_like(gray)
        binary_img[gray != 0] = 255
        if self.show_debug:
            cv2.imshow("binary", binary_img)
        return binary_img

    def _lane_offset(self, warped_img):
        """crop된 이미지를 원본 warped 이미지 좌표계로 변환할 때 필요한 X축 오프셋 계산
        
        왼쪽 차선: 0 (crop된 이미지가 이미 왼쪽)
        오른쪽 차선: 이미지 너비의 절반 (crop된 이미지를 오른쪽으로 이동)
        """
        return 0 if self.use_left_lane else warped_img.shape[1] // 2

    def _update_steering_standard(self, crop_width):
        """조향 기준점 업데이트 (crop된 이미지 너비 기준)
        
        Args:
            crop_width: crop된 이미지의 너비 (픽셀)
        """
        # 왼쪽/오른쪽 차선에 따라 다른 비율 적용
        ratio = self.left_ratio if self.use_left_lane else self.right_ratio
        ratio = min(max(ratio, 0.0), 1.0)  # 0.0~1.0 범위로 클램핑
        self.Steering_Standard = max(1, int(crop_width * ratio))

    def sliding_window(self, warped_img, croped_img, binary_img, croped_img_shape):
        """Sliding Window 알고리즘으로 차선의 중심점 추적
        
        이미지를 여러 개의 수평 윈도우로 나누고, 각 윈도우에서 히스토그램을 계산하여
        차선 픽셀이 가장 많은 영역의 중심을 찾습니다.
        
        Args:
            warped_img: 원근 변환된 원본 이미지 (시각화용)
            croped_img: 관심 영역으로 잘라낸 이미지 (시각화용)
            binary_img: 이진화된 이미지 (차선 검출용)
            croped_img_shape: 잘라낸 이미지의 (높이, 너비)
            
        Returns:
            avg_point: 가중 평균으로 계산된 차선 중심점 (X 좌표)
        """
        # 각 윈도우의 높이 계산
        crop_layer = max(1, croped_img_shape[0] // self.window_num)
        # 히스토그램에서 유효한 픽셀로 인정할 최소 임계값
        threshold = max(1, crop_layer // 2)
        histogram = []  # 각 윈도우의 X축 히스토그램
        indices = []  # 임계값을 넘는 X 좌표 인덱스
        middle_point = []  # 각 윈도우에서 검출된 차선 중심점
        lane_offset = self._lane_offset(warped_img)  # 원본 이미지 좌표계로 변환할 오프셋

        # 이미지 하단부터 상단으로 윈도우를 슬라이딩하며 차선 검출
        for i in range(0, self.window_num):
            # 원본 warped 이미지에서의 윈도우 Y 좌표
            original_window_top = warped_img.shape[0] - (i + 1) * crop_layer
            original_window_bottom = warped_img.shape[0] - (i) * crop_layer
            # crop된 이미지에서의 윈도우 Y 좌표
            cropped_window_top = croped_img_shape[0] - (i + 1) * crop_layer
            cropped_window_bottom = croped_img_shape[0] - (i) * crop_layer

            cropped_window_top = max(0, cropped_window_top)
            # X축 방향 히스토그램 계산 (각 X 좌표별 흰색 픽셀 개수)
            histogram.append(np.sum(binary_img[cropped_window_top:cropped_window_bottom, :], axis=0))
            # 임계값을 넘는 X 좌표들 찾기
            indices.append(np.where(histogram[i] > threshold)[0])

            try:
                # 검출된 인덱스들의 중심점 계산 (차선의 중심)
                current_middle = (min(indices[i]) + max(indices[i])) // 2
                middle_point.append(current_middle)
                cropped_window_left = current_middle - self.margin
                cropped_window_right = current_middle + self.margin

                # crop된 이미지에 윈도우 사각형 그리기 (빨간색)
                cv2.rectangle(
                    croped_img,
                    (cropped_window_left, cropped_window_top),
                    (cropped_window_right, cropped_window_bottom),
                    (0, 0, 255),
                    2,
                )
                # crop된 이미지에 차선 중심점 그리기 (초록색 원)
                cv2.circle(
                    croped_img,
                    (current_middle, (cropped_window_top + cropped_window_bottom) // 2),
                    3,
                    (0, 255, 0),
                    -1,
                )

                # 원본 warped 이미지 좌표계로 변환
                original_window_left = cropped_window_left + lane_offset
                original_window_right = cropped_window_right + lane_offset
                circle_point = current_middle + lane_offset

                # warped 이미지에도 동일하게 시각화
                cv2.rectangle(
                    warped_img,
                    (original_window_left, original_window_top),
                    (original_window_right, original_window_bottom),
                    (0, 0, 255),
                    2,
                )
                cv2.circle(
                    warped_img,
                    (circle_point, (original_window_top + original_window_bottom) // 2),
                    3,
                    (0, 255, 0),
                    -1,
                )
            except (ValueError, IndexError):
                # 차선 픽셀이 검출되지 않은 윈도우는 건너뛰기
                rospy.logwarn_once("Lane pixels not detected in some sliding windows.")

        # 가중 평균으로 최종 차선 중심점 계산
        # 하단 윈도우(가까운 거리)일수록 더 높은 가중치 부여
        try:
            weight_point = np.zeros_like(middle_point)
            for index, point in enumerate(middle_point):
                # index가 작을수록 (하단일수록) 가중치가 높음
                weight_point[index] = middle_point[0] - (middle_point[0] - point) * (len(middle_point) - index) / len(middle_point)
            avg_point = int(np.average(weight_point))
            self.previous_point = avg_point  # 성공 시 저장
        except Exception:
            # 차선 검출 실패 시 이전 프레임의 값 사용
            avg_point = self.previous_point

        # 조향 기준선 계산 (왼쪽 차선은 그대로, 오른쪽 차선은 반전)
        if self.use_left_lane:
            steering_standard = self.Steering_Standard
        else:
            steering_standard = croped_img_shape[1] - self.Steering_Standard

        # warped 이미지에 조향 기준 영역 표시 (파란색 사각형)
        cv2.rectangle(
            warped_img,
            (steering_standard, warped_img.shape[0] - croped_img_shape[0]),
            (warped_img.shape[1] - steering_standard, warped_img.shape[0]),
            (255, 0, 0),
            5,
        )
        # crop된 이미지에 조향 기준선 표시 (파란색 수직선)
        cv2.line(croped_img, (self.Steering_Standard, 0), (self.Steering_Standard, croped_img_shape[0]), (255, 0, 0), 3)
        
        # 디버그 정보 텍스트로 표시
        str_standard_point = f"standard x: {self.Steering_Standard}"
        str_avg_point = f"avg point: {avg_point}"
        cv2.putText(warped_img, str_standard_point, (warped_img.shape[1] // 8, warped_img.shape[0] // 8), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(warped_img, str_avg_point, (warped_img.shape[1] // 8, warped_img.shape[0] // 4), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        return avg_point

    def cal_angle_per_pixel(self, cv_img):
        """픽셀당 각도 해상도 계산
        
        이미지 너비를 기준으로 1픽셀이 몇 라디안에 해당하는지 계산합니다.
        (180도 = π 라디안을 이미지 너비로 나눔)
        
        Args:
            cv_img: 입력 이미지
            
        Returns:
            angle_resolution: 픽셀당 각도 (라디안)
        """
        return pi / cv_img.shape[1]

    def sense(self):
        """센서 데이터 수신 단계: ROS 이미지 메시지를 OpenCV 이미지로 변환
        
        Returns:
            cv_img: OpenCV 형식의 이미지, 변환 실패 시 None
        """
        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return None

        if self.show_debug:
            cv2.imshow("raw", cv_img)  # 원본 이미지 표시

        return cv_img

    def think(self, cv_img):
        """사고 단계: 이미지 처리 및 차선 검출
        
        Args:
            cv_img: OpenCV 이미지
            
        Returns:
            warped_img: 원근 변환된 이미지 (시각화용)
            croped_img: 관심 영역으로 잘라낸 이미지 (시각화용)
            avg_point: 검출된 차선의 중심점 X 좌표
            angle_resolution: 픽셀당 각도 해상도
        """
        # 1. 원근 변환 (Bird's Eye View)
        warped_img = self.birds_eyeview(cv_img)
        # 2. HSV 색상 필터링
        hsv_img = self.apply_mask(warped_img)
        # 3. 관심 영역 추출
        croped_img, croped_img_shape = self.crop_img(hsv_img)
        # 4. 조향 기준점 업데이트
        self._update_steering_standard(croped_img_shape[1])
        # 5. 이진화
        binary_img = self.binary(croped_img)
        # 6. Sliding Window로 차선 검출
        avg_point = self.sliding_window(warped_img, croped_img, binary_img, croped_img_shape)
        # 7. 각도 해상도 계산
        angle_resolution = self.cal_angle_per_pixel(cv_img)

        return warped_img, croped_img, avg_point, angle_resolution

    def _current_gain(self):
        """현재 사용할 조향 게인 결정
        
        steering_gain 파라미터가 설정되어 있으면 해당 값 사용,
        없으면 왼쪽/오른쪽 차선별 게인 사용
        
        Returns:
            gain: 조향 게인 값
        """
        gain_override = rospy.get_param("~steering_gain", self.gain_override)
        if gain_override is not None:
            return float(gain_override)
        return self.left_gain if self.use_left_lane else self.right_gain

    def act(self, cv_img, avg_point, angle_resolution):
        """행동 단계: 조향 각도 계산 및 Coss 명령 발행
        
        Args:
            cv_img: 원본 이미지 (시각화용)
            avg_point: 검출된 차선의 중심점 X 좌표
            angle_resolution: 픽셀당 각도 해상도
        """
        # 런타임에 파라미터 값 다시 읽기 (동적 변경 가능)
        enable_drive = bool(rospy.get_param("~enable_drive", self.enable_drive_default))

        if not enable_drive:
            # 주행 비활성화 시 조향각만 0으로 설정
            self.cmd_msg.cam_steer = 0.0
        else:
            # 주행 활성화 시 조향 각도 계산
            gain = self._current_gain()
            # 조향 각도 계산 (픽셀 오차를 라디안 각도로 변환)
            # 음수를 곱하는 이유: 차선이 오른쪽에 있으면 왼쪽으로 조향해야 함
            self.cmd_msg.cam_steer = gain * -angle_resolution * (avg_point - self.Steering_Standard)

        # 디버그 로그 출력
        rospy.loginfo(f"Angle Resolution: {angle_resolution}")
        rospy.loginfo(f"Avg Point - Steering Standard: {avg_point - self.Steering_Standard}")
        rospy.loginfo(f"Steering: {self.cmd_msg.cam_steer}")

        # 원본 이미지에 조향 정보 표시
        str_steering = f"steer: {round(self.cmd_msg.cam_steer, 2)}"
        if self.show_debug:
            cv2.putText(cv_img, str_steering, (cv_img.shape[1] * 5 // 8, cv_img.shape[0] // 8), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        # Coss 조향 명령 발행
        self.pub.publish(self.cmd_msg)

    def cam_CB(self, msg):
        """카메라 이미지 토픽 콜백 함수
        
        Args:
            msg: ROS Image 메시지
        """
        self.img_msg = msg
        self.cam_flag = True

    def run(self):
        """메인 루프: sense-think-act 사이클 실행"""
        # 카메라 이미지를 아직 받지 못했으면 대기
        if not self.cam_flag or self.img_msg is None:
            return

        # 1. Sense: 이미지 변환
        cv_img = self.sense()
        if cv_img is None:
            return

        # 2. Think: 차선 검출
        warped_img, croped_img, avg_point, angle_resolution = self.think(cv_img)
        # 3. Act: 조향 명령 발행
        self.act(cv_img, avg_point, angle_resolution)

        # 디버그 창 표시
        if self.show_debug:
            cv2.imshow("warped", warped_img)
            cv2.imshow("cropped", croped_img)
            cv2.waitKey(1)


def main():
    """메인 함수: 노드 실행 및 종료 처리"""
    try:
        # ImageProcessor 객체 생성 (노드 초기화)
        image_processor = ImageProcessor()
        # 30Hz로 루프 실행
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            image_processor.run()  # sense-think-act 사이클 실행
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        # 종료 시 모든 OpenCV 창 닫기
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()