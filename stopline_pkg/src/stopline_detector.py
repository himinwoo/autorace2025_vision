#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import numpy as np
import cv2

class StopLineDetector:
    """
    정지선 감지 클래스
    히스토그램 기반으로 이미지에서 정지선을 감지합니다.
    """
    def __init__(self, down_hist_start_line=400, stopline_threshold=10, count_threshold=10, histogram_threshold=600,
                 use_hough=True, hough_threshold=120, hough_min_line_length=150, hough_max_line_gap=100, 
                 hough_angle_threshold=30, hough_width_ratio=0.3):
        """
        정지선 감지기 초기화
        
        Args:
            down_hist_start_line: 히스토그램 검색 시작 라인 (이미지 하단부)
            stopline_threshold: 정지선 인식 너비 임계값
            count_threshold: 정지선 감지 확정을 위한 연속 감지 횟수
            histogram_threshold: 정지선 판단을 위한 히스토그램 임계값
            use_hough: 허프 변환 사용 여부
            hough_threshold: 허프 변환 임계값 (투표 수)
            hough_min_line_length: 허프 변환 최소 선 길이
            hough_max_line_gap: 허프 변환 최대 선 간격
            hough_angle_threshold: 허프 변환 각도 임계값 (도)
            hough_width_ratio: 허프 변환 너비 비율 (0.0~1.0)
        """
        self.down_hist_start_line = down_hist_start_line
        self.stopline_threshold = stopline_threshold
        self.count_threshold = count_threshold
        self.histogram_threshold = histogram_threshold
        
        # 허프 변환 파라미터
        self.use_hough = use_hough
        self.hough_threshold = hough_threshold
        self.hough_min_line_length = hough_min_line_length
        self.hough_max_line_gap = hough_max_line_gap
        self.hough_angle_threshold = hough_angle_threshold
        self.hough_width_ratio = hough_width_ratio
        
        # 정지선 감지 카운터
        self.stopline_count = 0
        
    def detect(self, bin_img):
        """
        이진화된 이미지에서 정지선을 감지합니다.
        
        Args:
            bin_img: 이진화된 이미지 (numpy array)
            
        Returns:
            tuple: (detected, stopline_indices, stopline_count)
                - detected: 정지선이 감지되었는지 여부 (bool)
                - stopline_indices: 정지선의 y 좌표 인덱스 (numpy array or None)
                - stopline_count: 현재 정지선 감지 카운트
        """
        # y축 히스토그램 계산 (각 행의 흰색 픽셀 수 합산)
        histogram_y = np.sum(bin_img, axis=1)
        
        # 이미지 하단부에서 정지선 탐색
        down_hist = histogram_y[self.down_hist_start_line:]
        stopline_indices = np.where(down_hist > self.histogram_threshold)[0] + self.down_hist_start_line
        
        detected = False
        histogram_detected = False
        
        # 방법 1: 기존 히스토그램 방식 (수평선에 강함)
        try:
            stopline_diff = stopline_indices[-1] - stopline_indices[0]
            if self.stopline_threshold < stopline_diff:
                histogram_detected = True
        except:
            stopline_indices = None
        
        # 방법 2: 허프 변환 (대각선에도 강함)
        hough_detected = False
        if self.use_hough:
            bin_img_uint8 = (bin_img * 255).astype(np.uint8)
            
            # 관심 영역만 검사 (하단부)
            roi = bin_img_uint8[self.down_hist_start_line:, :]
            
            # 허프 변환으로 직선 검출
            lines = cv2.HoughLinesP(roi, rho=1, theta=np.pi/180, threshold=self.hough_threshold, 
                                    minLineLength=self.hough_min_line_length, maxLineGap=self.hough_max_line_gap)
            
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    # 대략 수평에 가까운 선만 고려
                    angle = np.abs(np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi)
                    if angle < self.hough_angle_threshold:  # 수평에서 지정된 각도 이내
                        # 선이 이미지 너비의 일정 비율 이상이면 정지선으로 판단
                        line_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                        if line_length > roi.shape[1] * self.hough_width_ratio:
                            hough_detected = True
                            # stopline_indices 업데이트 (허프 변환 결과 반영)
                            y_start = min(y1, y2) + self.down_hist_start_line
                            y_end = max(y1, y2) + self.down_hist_start_line
                            if stopline_indices is None or len(stopline_indices) == 0:
                                stopline_indices = np.array([y_start, y_end])
                            break
        
        # 둘 중 하나라도 감지되면 카운트 증가
        if histogram_detected or hough_detected:
            self.stopline_count += 1
            
            # 연속 감지 횟수가 임계값을 넘으면 정지선 감지 확정
            if self.stopline_count >= self.count_threshold:
                detected = True
        else:
            # 정지선을 찾지 못한 경우 카운터 리셋
            self.stopline_count = 0
            stopline_indices = None
            
        return detected, stopline_indices, self.stopline_count
    
    def reset_count(self):
        """
        정지선 감지 카운터를 리셋합니다.
        """
        self.stopline_count = 0
        
    def draw_stopline(self, img, stopline_indices, color=(0, 0, 255), thickness=3):
        """
        이미지에 정지선 영역을 표시합니다.
        
        Args:
            img: 표시할 이미지
            stopline_indices: 정지선의 y 좌표 인덱스
            color: 표시할 색상 (BGR)
            thickness: 선 두께
            
        Returns:
            img: 정지선이 표시된 이미지
        """
        if stopline_indices is not None and len(stopline_indices) > 0:
            try:
                x = img.shape[1]
                cv2.rectangle(img, 
                            [0, stopline_indices[0]], 
                            [x, stopline_indices[-1]], 
                            color, 
                            thickness)
            except:
                pass
        return img
