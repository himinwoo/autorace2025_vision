#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import numpy as np
import cv2

class StopLineDetector:
    """
    정지선 감지 클래스
    히스토그램 기반으로 이미지에서 정지선을 감지합니다.
    """
    def __init__(self, down_hist_start_line=400, stopline_threshold=10, count_threshold=10):
        """
        정지선 감지기 초기화
        
        Args:
            down_hist_start_line: 히스토그램 검색 시작 라인 (이미지 하단부)
            stopline_threshold: 정지선 인식 너비 임계값
            count_threshold: 정지선 감지 확정을 위한 연속 감지 횟수
        """
        self.down_hist_start_line = down_hist_start_line
        self.stopline_threshold = stopline_threshold
        self.count_threshold = count_threshold
        
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
        stopline_indices = np.where(down_hist > 600)[0] + self.down_hist_start_line
        
        detected = False
        
        try:
            stopline_diff = stopline_indices[-1] - stopline_indices[0]
            if self.stopline_threshold < stopline_diff:
                self.stopline_count += 1
                
                # 연속 감지 횟수가 임계값을 넘으면 정지선 감지 확정
                if self.stopline_count >= self.count_threshold:
                    detected = True
        except:
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
