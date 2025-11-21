#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class ArrowSignDetector:
    def __init__(self):
        rospy.init_node('arrow_sign_detector')
        self.bridge = CvBridge()
        
        # Subscriber & Publisher
        self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, self.callback)
        self.direction_pub = rospy.Publisher('camera/sign/direction', String, queue_size=1)
        
        # 파란색 표지판 HSV 범위
        self.lower_blue = np.array([100, 100, 50])
        self.upper_blue = np.array([130, 255, 255])
        
        # 파라미터
        self.min_area = 1000  # 최소 표지판 면적
        self.threshold_value = 180  # 흰색 검출 threshold
        self.debug_mode = True  # 디버깅 모드
        
        rospy.loginfo("Arrow Sign Detector initialized")
        rospy.loginfo("Subscribing to: /usb_cam/image_rect_color")
        rospy.loginfo("Publishing to: /camera/sign/direction")

    def detect_arrow_direction(self, image):
        """파란색 표지판에서 흰색 화살표 방향 검출"""
        # 디버깅용 이미지
        debug_image = image.copy() if self.debug_mode else None
        
        # 파란색 표지판 검출
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        
        # 노이즈 제거
        kernel = np.ones((5, 5), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
        
        if self.debug_mode:
            cv2.imshow("1. Blue Mask", mask_blue)
        
        # 표지판 찾기
        contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            if self.debug_mode:
                cv2.imshow("0. Original", debug_image)
                cv2.waitKey(1)
            return None
        
        # 가장 큰 contour 선택
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        if area < self.min_area:
            if self.debug_mode:
                cv2.imshow("0. Original", debug_image)
                cv2.waitKey(1)
            return None
        
        # ROI 추출
        x, y, w, h = cv2.boundingRect(largest)
        
        if self.debug_mode:
            cv2.rectangle(debug_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(debug_image, f"{w}x{h}", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        roi = image[y:y+h, x:x+w]
        
        if self.debug_mode:
            cv2.imshow("2. ROI", roi)
        
        # 흰색 화살표 검출 (그레이스케일 + threshold)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        if self.debug_mode:
            cv2.imshow("3. Gray", gray)
        
        _, white_mask = cv2.threshold(gray, self.threshold_value, 255, cv2.THRESH_BINARY)
        
        # 노이즈 제거
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        
        if self.debug_mode:
            cv2.imshow("4. White Mask", white_mask)
        
        # 화살표 픽셀의 좌우 분포 계산
        h_roi, w_roi = white_mask.shape
        left_half = white_mask[:, :w_roi//2]
        right_half = white_mask[:, w_roi//2:]
        
        left_pixels = np.sum(left_half == 255)
        right_pixels = np.sum(right_half == 255)
        
        # 디버깅: 좌우 분할 시각화
        if self.debug_mode:
            split_visual = cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)
            cv2.line(split_visual, (w_roi//2, 0), (w_roi//2, h_roi), (0, 0, 255), 2)
            cv2.imshow("5. Split", split_visual)
        
        # 화살촉 방향 판단
        total_pixels = left_pixels + right_pixels
        
        if total_pixels < 100:
            if self.debug_mode:
                cv2.putText(debug_image, "No arrow", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow("0. Original", debug_image)
                cv2.waitKey(1)
            return None
        
        # 좌우 차이가 20% 이상일 때만 방향 결정
        if left_pixels > right_pixels * 1.2:
            direction = "left"
        elif right_pixels > left_pixels * 1.2:
            direction = "right"
        else:
            direction = None
        
        # 디버깅: 결과 표시
        if self.debug_mode:
            if direction:
                color = (0, 255, 0)
                cv2.putText(debug_image, f"Direction: {direction}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            else:
                color = (0, 255, 255)
                cv2.putText(debug_image, "Unclear", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            
            cv2.putText(debug_image, f"L:{left_pixels} R:{right_pixels}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.imshow("0. Original", debug_image)
            cv2.waitKey(1)
        
        return direction
    
    def callback(self, msg):
        """이미지 콜백"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            direction = self.detect_arrow_direction(cv_image)
            
            if direction:
                self.direction_pub.publish(direction)
                rospy.loginfo(f"Detected: {direction}")
        except Exception as e:
            rospy.logerr(f"Error: {e}")
    
    def run(self):
        """노드 실행"""
        rospy.loginfo("Starting arrow sign detector...")
        try:
            rospy.spin()
        finally:
            if self.debug_mode:
                cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = ArrowSignDetector()
        detector.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
