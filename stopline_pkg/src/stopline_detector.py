#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import numpy as np
import cv2


class StopLineDetector:
    """
    정지선 감지 클래스
    히스토그램 기반으로 이미지에서 정지선을 감지합니다.
    개선사항: 허프 변환 다중 라인 검증, 시간적 일관성 버퍼, 원형 영역 필터링
    """
    def __init__(self, down_hist_start_line=400, stopline_threshold=10, count_threshold=15, histogram_threshold=800,
                 use_hough=True, hough_threshold=100, hough_min_line_length=200, hough_max_line_gap=50, 
                 hough_angle_threshold=15, hough_width_ratio=0.4, hough_consistency_threshold=0.7,
                 otsu_threshold_offset=50, history_size=5):
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
        self.hough_consistency_threshold = hough_consistency_threshold

        # Otsu offset
        self.otsu_threshold_offset = otsu_threshold_offset

        # 시간적 일관성
        self.line_history = []
        self.history_size = history_size

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

        # 방법 1: 히스토그램 방식
        try:
            stopline_diff = stopline_indices[-1] - stopline_indices[0]
            if self.stopline_threshold < stopline_diff:
                histogram_detected = True
        except Exception:
            stopline_indices = None

        # 방법 2: 개선된 허프 변환
        hough_detected, hough_line_pos = self._detect_hough_lines(bin_img)

        # AND 조건: 히스토그램과 허프가 모두 감지한 경우에만 보수적으로 판단
        # if histogram_detected and hough_detected:
        if histogram_detected:
            # # 시간적 일관성 검증
            # if self._verify_temporal_consistency(hough_line_pos):
            #     self.stopline_count += 1
            self.stopline_count += 1

            if self.stopline_count >= self.count_threshold:
                detected = True
        else:
            # 점진적 감소
            self.stopline_count = max(0, self.stopline_count - 1)

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

    def _detect_hough_lines(self, bin_img):
        """
        개선된 허프 라인 검출 - 다중 라인 검증 및 위치 클러스터링
        Returns: (bool detected, float avg_y_pos or None)
        """
        if not self.use_hough:
            return False, None

        bin_img_uint8 = (bin_img * 255).astype(np.uint8)
        roi = bin_img_uint8[self.down_hist_start_line:, :]

        lines = cv2.HoughLinesP(
            roi, rho=1, theta=np.pi/180,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap
        )

        if lines is None:
            return False, None

        valid_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = np.abs(np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi)
            if angle > self.hough_angle_threshold:
                continue

            line_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            if line_length < roi.shape[1] * self.hough_width_ratio:
                continue

            y_pos = (y1 + y2) / 2.0 + self.down_hist_start_line

            valid_lines.append({'y_pos': y_pos, 'length': line_length, 'angle': angle})

        # 다중 라인 필요
        if len(valid_lines) < 3:
            return False, None

        y_positions = [l['y_pos'] for l in valid_lines]
        y_std = np.std(y_positions)

        if y_std < 10:
            avg_y_pos = np.mean(y_positions)
            return True, avg_y_pos

        return False, None

    def _verify_temporal_consistency(self, current_line_pos):
        """
        시간적 일관성 검증
        """
        if current_line_pos is None:
            return False

        self.line_history.append(current_line_pos)
        if len(self.line_history) > self.history_size:
            self.line_history.pop(0)

        if len(self.line_history) < 3:
            return True

        positions = np.array(self.line_history)
        std_dev = np.std(positions)

        return std_dev < 15

    def _filter_circular_regions(self, binary_img):
        """
        원형 영역 제거 (빛 반사는 동그랗게 나타남)
        """
        contours, _ = cv2.findContours(
            binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        filtered_mask = np.zeros_like(binary_img)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 100:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue

            circularity = 4 * np.pi * area / (perimeter * perimeter)

            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h if h > 0 else 0

            if circularity < 0.6 and aspect_ratio > 3.0:
                cv2.drawContours(filtered_mask, [cnt], -1, 255, -1)

        return (filtered_mask > 0).astype(np.uint8)
