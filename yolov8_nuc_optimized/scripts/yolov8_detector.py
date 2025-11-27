#!/usr/bin/env python3
"""
YOLOv8 Detector Node - Intel NUC Optimized
최적화된 성능과 높은 가독성을 위해 설계된 YOLOv8 객체 검출 노드
"""

import rospy
import cv2
import numpy as np
import json
import time
from pathlib import Path
from typing import List, Dict, Tuple, Optional

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from coss_msgs.msg import Coss

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False
    rospy.logwarn("Ultralytics not installed. Install with: pip3 install ultralytics")


class YOLOv8Detector:
    """
    Intel NUC에 최적화된 YOLOv8 객체 검출기
    
    주요 기능:
    - OpenVINO 가속 지원
    - 효율적인 메모리 관리
    - 실시간 성능 모니터링
    - 유연한 설정 옵션
    """
    
    def __init__(self):
        """노드 초기화 및 설정 로드"""
        rospy.init_node('yolov8_detector', anonymous=True)
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 설정 로드
        self._load_config()
        
        # 모델 로드
        self.model = self._load_model()
        
        # 성능 측정 변수
        self.fps_counter = FPSCounter()
        
        # 클래스 검출 안정화를 위한 변수
        self.detection_threshold = rospy.get_param('~detection_threshold', 5)  # 연속 검출 횟수
        self.current_class = -1  # 현재 추적 중인 클래스
        self.class_count = 0  # 연속 검출 카운터
        self.last_published_class = -1  # 마지막으로 퍼블리시한 클래스
        
        # ROS 퍼블리셔 설정
        # Coss 메시지로 클래스 퍼블리시
        self.coss_pub = rospy.Publisher(
            '/camera/class',
            Coss,
            queue_size=1
        )
        # 디버깅용 이미지 퍼블리셔
        self.image_pub = rospy.Publisher(
            self.config['topics']['output_image'],
            Image,
            queue_size=1
        )
        
        # ROS 서브스크라이버 설정
        self.image_sub = rospy.Subscriber(
            self.config['topics']['input_image'],
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2**24  # 버퍼 크기 증가로 지연 감소
        )
        
        rospy.loginfo("YOLOv8 Detector initialized successfully")
        rospy.loginfo(f"Model: {self.config['model']['path']}")
        rospy.loginfo(f"OpenVINO: {'Enabled' if self.config['performance']['use_openvino'] else 'Disabled'}")
    
    def _load_config(self) -> None:
        """YAML 설정 파일에서 파라미터 로드"""
        self.config = {
            'model': {
                'path': rospy.get_param('~model_path', 'sign.pt'),
                'input_size': rospy.get_param('~input_size', 640),
                'format': rospy.get_param('~model_format', 'pt')
            },
            'performance': {
                'use_openvino': rospy.get_param('~use_openvino', True),
                'use_fp16': rospy.get_param('~use_fp16', True),
                'num_threads': rospy.get_param('~num_threads', 4),
                'device': rospy.get_param('~device', 'CPU')
            },
            'detection': {
                'confidence_threshold': rospy.get_param('~confidence_threshold', 0.35),
                'iou_threshold': rospy.get_param('~iou_threshold', 0.45),
                'max_detections': rospy.get_param('~max_detections', 100),
                'classes': rospy.get_param('~classes', [])
            },
            'topics': {
                'input_image': rospy.get_param('~input_topic', '/camera/image_raw'),
                'output_image': rospy.get_param('~output_topic', '/yolov8/detections'),
                'detection_info': rospy.get_param('~info_topic', '/yolov8/detection_info')
            },
            'visualization': {
                'enable': rospy.get_param('~enable_visualization', True),
                'show_labels': rospy.get_param('~show_labels', True),
                'show_confidence': rospy.get_param('~show_confidence', True),
                'box_thickness': rospy.get_param('~box_thickness', 2),
                'font_scale': rospy.get_param('~font_scale', 0.6)
            },
            'debug': {
                'enable': rospy.get_param('~debug', False),
                'show_fps': rospy.get_param('~show_fps', True),
                'log_detections': rospy.get_param('~log_detections', False)
            }
        }
    
    def _load_model(self) -> Optional[YOLO]:
        """
        YOLOv8 모델 로드 및 최적화
        
        Returns:
            YOLO 모델 객체 또는 None
        """
        if not ULTRALYTICS_AVAILABLE:
            rospy.logerr("Ultralytics not available. Cannot load model.")
            return None
        
        try:
            model_path = self.config['model']['path']
            
            # 모델 파일 존재 확인
            if not Path(model_path).exists():
                rospy.logwarn(f"Model not found at {model_path}. Downloading default model...")
                model_path = 'yolov8n.pt'
            
            # 모델 로드
            model = YOLO(model_path)
            rospy.loginfo(f"Model loaded: {model_path}")
            
            # OpenVINO 최적화 (Intel NUC용)
            if self.config['performance']['use_openvino']:
                try:
                    # ONNX로 변환 후 OpenVINO로 변환
                    onnx_path = model_path.replace('.pt', '.onnx')
                    if not Path(onnx_path).exists():
                        rospy.loginfo("Exporting model to ONNX format...")
                        model.export(format='onnx', dynamic=False, simplify=True)
                    
                    # OpenVINO 형식으로 변환
                    openvino_path = model_path.replace('.pt', '_openvino_model')
                    if not Path(openvino_path).exists():
                        rospy.loginfo("Converting to OpenVINO format...")
                        model.export(format='openvino', half=self.config['performance']['use_fp16'])
                    
                    # OpenVINO 모델 로드
                    model = YOLO(openvino_path)
                    rospy.loginfo("OpenVINO optimization enabled")
                except Exception as e:
                    rospy.logwarn(f"OpenVINO optimization failed: {e}. Using default model.")
            
            # 워밍업 (첫 실행 지연 제거)
            rospy.loginfo("Warming up model...")
            dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
            _ = model(dummy_img, verbose=False, task='detect')
            rospy.loginfo("Model ready for inference")
            
            return model
            
        except Exception as e:
            rospy.logerr(f"Failed to load model: {e}")
            return None
    
    def _image_callback(self, msg: Image) -> None:
        """
        이미지 토픽 콜백 함수
        
        Args:
            msg: ROS Image 메시지
        """
        if self.model is None:
            return
        
        try:
            # ROS Image를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 이미지 전처리 (선명도 및 대비 향상)
            preprocessed_image = self._preprocess_image(cv_image)
            
            # 객체 검출 수행
            detections = self._detect_objects(preprocessed_image)
            
            # 검출된 클래스를 Coss 메시지로 퍼블리시 (안정화 처리)
            self._update_class_detection(detections)
            
            # 시각화 및 이미지 퍼블리시 (디버깅용)
            if self.config['visualization']['enable']:
                vis_image = self._visualize_detections(cv_image, detections)
                self._publish_image(vis_image)
            
            # FPS 업데이트
            self.fps_counter.update()
            
            # 디버그 정보 출력
            if self.config['debug']['show_fps']:
                fps = self.fps_counter.get_fps()
                if fps > 0:
                    rospy.loginfo_throttle(1.0, f"FPS: {fps:.1f}")
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Error in image callback: {e}")
    
    def _preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """
        이미지 전처리 - 파란색 표지판 영역 추출 및 향상
        
        Args:
            image: 원본 이미지 (BGR)
            
        Returns:
            전처리된 이미지
        """
        # HSV로 변환
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 파란색 범위 설정 (제공된 샘플 기반)
        # H: 53~94, S: 36~63, V: 150~186
        lower_blue = np.array([50, 30, 145])
        upper_blue = np.array([100, 70, 190])
        
        # 파란색 마스크 생성
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # 모폴로지 연산으로 노이즈 제거 및 영역 강화
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        
        # 마스크를 3채널로 확장
        blue_mask_3ch = cv2.cvtColor(blue_mask, cv2.COLOR_GRAY2BGR)
        
        # 파란색 영역만 추출
        blue_region = cv2.bitwise_and(image, blue_mask_3ch)
        
        # 파란색 영역 강조 (원본 이미지와 블렌딩)
        # 파란색 영역은 밝게, 나머지는 어둡게
        enhanced = cv2.addWeighted(image, 0.3, blue_region, 2.0, 0)
        
        # CLAHE로 대비 향상
        lab = cv2.cvtColor(enhanced, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        enhanced = cv2.merge([l, a, b])
        enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        
        # 선명도 향상
        gaussian = cv2.GaussianBlur(enhanced, (0, 0), 2.0)
        sharpened = cv2.addWeighted(enhanced, 1.5, gaussian, -0.5, 0)
        
        return sharpened
    
    def _detect_objects(self, image: np.ndarray) -> List[Dict]:
        """
        이미지에서 객체 검출
        
        Args:
            image: OpenCV 이미지 (BGR)
            
        Returns:
            검출된 객체 정보 리스트
        """
        try:
            # YOLOv8 추론
            results = self.model(
                image,
                conf=self.config['detection']['confidence_threshold'],
                iou=self.config['detection']['iou_threshold'],
                max_det=self.config['detection']['max_detections'],
                classes=self.config['detection']['classes'] if self.config['detection']['classes'] else None,
                verbose=False,
                imgsz=self.config['model']['input_size'],
                task='detect'  # 명시적으로 task 지정하여 경고 제거
            )
            
            # 결과 파싱
            detections = []
            if len(results) > 0 and results[0].boxes is not None:
                boxes = results[0].boxes
                
                for i in range(len(boxes)):
                    # 바운딩 박스 좌표 (xyxy)
                    x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy()
                    
                    # 신뢰도
                    confidence = float(boxes.conf[i].cpu().numpy())
                    
                    # 클래스 ID 및 이름
                    class_id = int(boxes.cls[i].cpu().numpy())
                    class_name = self.model.names[class_id]
                    
                    detection = {
                        'bbox': [float(x1), float(y1), float(x2), float(y2)],
                        'confidence': confidence,
                        'class_id': class_id,
                        'class_name': class_name
                    }
                    detections.append(detection)
            
            return detections
            
        except Exception as e:
            rospy.logerr(f"Detection error: {e}")
            return []
    
    def _update_class_detection(self, detections: List[Dict]) -> None:
        """
        검출된 클래스를 추적하고 일정 횟수 이상 연속 검출 시 퍼블리시
        
        Args:
            detections: 검출 정보 리스트
        """
        if detections:
            detected_class = detections[0]['class_id']
            
            # 동일한 클래스가 연속으로 검출되는지 확인
            if detected_class == self.current_class:
                self.class_count += 1
            else:
                # 다른 클래스가 검출되면 카운터 리셋
                self.current_class = detected_class
                self.class_count = 1
            
            # 임계값에 도달하면 퍼블리시 (이전 클래스와 상관없이)
            if self.class_count >= self.detection_threshold:
                self._publish_class(self.current_class, detections[0]['class_name'])
                self.last_published_class = self.current_class
        else:
            # 검출이 없으면 -1 퍼블리시
            self._publish_class(-1, "no_detection")
            self.current_class = -1
            self.class_count = 0
            self.ㄴ = -1
    
    def _publish_class(self, class_id: int, class_name: str) -> None:
        """
        검출된 클래스를 Coss 메시지로 퍼블리시
        
        Args:
            class_id: 클래스 ID
            class_name: 클래스 이름
        """
        coss_msg = Coss()
        coss_msg.cam_class = class_id
        
        self.coss_pub.publish(coss_msg)
        
        # 디버그 로그
        if self.config['debug']['log_detections']:
            rospy.loginfo(f"Published class: {class_name} (ID: {class_id})")
    
    def _visualize_detections(self, image: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """
        검출 결과 시각화
        
        Args:
            image: 원본 이미지
            detections: 검출 정보 리스트
            
        Returns:
            시각화된 이미지
        """
        vis_image = image.copy()
        
        for det in detections:
            # 바운딩 박스 그리기
            x1, y1, x2, y2 = map(int, det['bbox'])
            color = self._get_color(det['class_id'])
            
            cv2.rectangle(
                vis_image,
                (x1, y1),
                (x2, y2),
                color,
                self.config['visualization']['box_thickness']
            )
            
            # 라벨 및 신뢰도 표시
            if self.config['visualization']['show_labels']:
                label = det['class_name']
                if self.config['visualization']['show_confidence']:
                    label += f" {det['confidence']:.2f}"
                
                # 라벨 배경
                (text_width, text_height), _ = cv2.getTextSize(
                    label,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    self.config['visualization']['font_scale'],
                    2
                )
                
                cv2.rectangle(
                    vis_image,
                    (x1, y1 - text_height - 10),
                    (x1 + text_width, y1),
                    color,
                    -1
                )
                
                # 라벨 텍스트
                cv2.putText(
                    vis_image,
                    label,
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    self.config['visualization']['font_scale'],
                    (255, 255, 255),
                    2
                )
        
        # FPS 표시
        if self.config['debug']['show_fps']:
            fps = self.fps_counter.get_fps()
            cv2.putText(
                vis_image,
                f"FPS: {fps:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )
        
        return vis_image
    
    def _publish_image(self, image: np.ndarray) -> None:
        """
        이미지 퍼블리시 (디버깅용)
        
        Args:
            image: OpenCV 이미지
        """
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self.image_pub.publish(msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to publish image: {e}")
    
    @staticmethod
    def _get_color(class_id: int) -> Tuple[int, int, int]:
        """
        클래스 ID에 따른 색상 반환
        
        Args:
            class_id: 클래스 ID
            
        Returns:
            BGR 색상 튜플
        """
        np.random.seed(class_id)
        return tuple(map(int, np.random.randint(0, 255, 3)))
    
    def run(self) -> None:
        """노드 실행"""
        rospy.spin()


class FPSCounter:
    """FPS 계산 유틸리티 클래스"""
    
    def __init__(self, window_size: int = 30):
        """
        Args:
            window_size: 평균 계산에 사용할 프레임 수
        """
        self.window_size = window_size
        self.timestamps = []
        self.last_time = time.time()
    
    def update(self) -> None:
        """타임스탬프 업데이트"""
        current_time = time.time()
        self.timestamps.append(current_time)
        
        # 윈도우 크기 유지
        if len(self.timestamps) > self.window_size:
            self.timestamps.pop(0)
        
        self.last_time = current_time
    
    def get_fps(self) -> float:
        """
        현재 FPS 반환
        
        Returns:
            초당 프레임 수
        """
        if len(self.timestamps) < 2:
            return 0.0
        
        time_diff = self.timestamps[-1] - self.timestamps[0]
        if time_diff > 0:
            return (len(self.timestamps) - 1) / time_diff
        return 0.0


def main():
    """메인 함수"""
    try:
        detector = YOLOv8Detector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")


if __name__ == '__main__':
    main()
