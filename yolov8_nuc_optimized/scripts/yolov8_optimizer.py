#!/usr/bin/env python3
"""
YOLOv8 Model Optimizer
YOLOv8 모델을 Intel NUC에 최적화된 형식으로 변환하는 유틸리티
"""

import argparse
import sys
from pathlib import Path

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False
    print("ERROR: Ultralytics not installed. Install with: pip3 install ultralytics")
    sys.exit(1)


class ModelOptimizer:
    """
    YOLOv8 모델 최적화 클래스
    
    지원 형식:
    - ONNX: 범용 최적화 포맷
    - OpenVINO: Intel 하드웨어 최적화
    - TensorRT: NVIDIA GPU 최적화 (선택사항)
    """
    
    def __init__(self, model_path: str):
        """
        Args:
            model_path: YOLOv8 모델 경로 (.pt 파일)
        """
        self.model_path = Path(model_path)
        
        if not self.model_path.exists():
            raise FileNotFoundError(f"Model not found: {model_path}")
        
        print(f"Loading model: {self.model_path}")
        self.model = YOLO(str(self.model_path))
        print("Model loaded successfully")
    
    def export_onnx(self, dynamic: bool = False, simplify: bool = True) -> str:
        """
        ONNX 형식으로 변환
        
        Args:
            dynamic: 동적 배치 크기 지원 여부
            simplify: ONNX 그래프 단순화 여부
            
        Returns:
            변환된 모델 경로
        """
        print("\n[1/1] Exporting to ONNX format...")
        print(f"  - Dynamic batch size: {dynamic}")
        print(f"  - Simplify graph: {simplify}")
        
        output_path = self.model.export(
            format='onnx',
            dynamic=dynamic,
            simplify=simplify
        )
        
        print(f"✓ ONNX export complete: {output_path}")
        return output_path
    
    def export_openvino(self, half: bool = True, int8: bool = False) -> str:
        """
        OpenVINO 형식으로 변환 (Intel NUC 최적화)
        
        Args:
            half: FP16 정밀도 사용 (속도 향상)
            int8: INT8 양자화 사용 (최대 속도, 정확도 약간 감소)
            
        Returns:
            변환된 모델 경로
        """
        print("\n[1/2] Exporting to ONNX format (intermediate step)...")
        onnx_path = self.export_onnx(dynamic=False, simplify=True)
        
        print("\n[2/2] Converting to OpenVINO format...")
        print(f"  - FP16 precision: {half}")
        print(f"  - INT8 quantization: {int8}")
        
        output_path = self.model.export(
            format='openvino',
            half=half,
            int8=int8
        )
        
        print(f"✓ OpenVINO export complete: {output_path}")
        self._print_openvino_info(output_path)
        return output_path
    
    def export_tensorrt(self, half: bool = True, workspace: int = 4) -> str:
        """
        TensorRT 형식으로 변환 (NVIDIA GPU 최적화)
        
        Args:
            half: FP16 정밀도 사용
            workspace: GPU 메모리 워크스페이스 (GB)
            
        Returns:
            변환된 모델 경로
        """
        print("\n[1/1] Exporting to TensorRT format...")
        print(f"  - FP16 precision: {half}")
        print(f"  - Workspace: {workspace} GB")
        
        try:
            output_path = self.model.export(
                format='engine',
                half=half,
                workspace=workspace
            )
            print(f"✓ TensorRT export complete: {output_path}")
            return output_path
        except Exception as e:
            print(f"✗ TensorRT export failed: {e}")
            print("Note: TensorRT requires NVIDIA GPU and proper installation")
            return None
    
    def benchmark(self, format: str = 'pt', imgsz: int = 640) -> None:
        """
        모델 벤치마크 수행
        
        Args:
            format: 벤치마크할 모델 형식
            imgsz: 입력 이미지 크기
        """
        print(f"\n=== Benchmarking {format.upper()} model ===")
        print(f"Image size: {imgsz}x{imgsz}")
        
        try:
            # 벤치마크 실행
            self.model.benchmark(
                data='coco8.yaml',
                imgsz=imgsz,
                device='cpu'
            )
        except Exception as e:
            print(f"Benchmark failed: {e}")
    
    @staticmethod
    def _print_openvino_info(model_path: str) -> None:
        """OpenVINO 모델 정보 출력"""
        model_path = Path(model_path)
        
        print("\n=== OpenVINO Model Info ===")
        print(f"Model directory: {model_path}")
        
        if model_path.is_dir():
            xml_files = list(model_path.glob("*.xml"))
            bin_files = list(model_path.glob("*.bin"))
            
            if xml_files:
                print(f"XML file: {xml_files[0].name}")
            if bin_files:
                print(f"BIN file: {bin_files[0].name}")
                file_size = bin_files[0].stat().st_size / (1024 * 1024)
                print(f"Model size: {file_size:.2f} MB")
    
    @staticmethod
    def compare_models(original_path: str, optimized_path: str) -> None:
        """
        원본 모델과 최적화된 모델 비교
        
        Args:
            original_path: 원본 모델 경로
            optimized_path: 최적화된 모델 경로
        """
        print("\n=== Model Comparison ===")
        
        original_size = Path(original_path).stat().st_size / (1024 * 1024)
        print(f"Original model: {original_size:.2f} MB")
        
        optimized_path = Path(optimized_path)
        if optimized_path.is_file():
            optimized_size = optimized_path.stat().st_size / (1024 * 1024)
            print(f"Optimized model: {optimized_size:.2f} MB")
            print(f"Size reduction: {(1 - optimized_size/original_size) * 100:.1f}%")
        elif optimized_path.is_dir():
            total_size = sum(f.stat().st_size for f in optimized_path.rglob('*') if f.is_file())
            total_size_mb = total_size / (1024 * 1024)
            print(f"Optimized model (dir): {total_size_mb:.2f} MB")


def main():
    """메인 함수"""
    parser = argparse.ArgumentParser(
        description="YOLOv8 Model Optimizer for Intel NUC",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # ONNX 변환
  python3 yolov8_optimizer.py yolov8n.pt --format onnx
  
  # OpenVINO 변환 (FP16)
  python3 yolov8_optimizer.py yolov8n.pt --format openvino --fp16
  
  # OpenVINO 변환 (INT8, 최고 속도)
  python3 yolov8_optimizer.py yolov8n.pt --format openvino --int8
  
  # 벤치마크 수행
  python3 yolov8_optimizer.py yolov8n.pt --benchmark
        """
    )
    
    parser.add_argument(
        'model',
        type=str,
        help='YOLOv8 model path (.pt file)'
    )
    
    parser.add_argument(
        '--format',
        type=str,
        choices=['onnx', 'openvino', 'tensorrt'],
        default='openvino',
        help='Export format (default: openvino)'
    )
    
    parser.add_argument(
        '--fp16',
        action='store_true',
        help='Use FP16 precision (faster, slight accuracy loss)'
    )
    
    parser.add_argument(
        '--int8',
        action='store_true',
        help='Use INT8 quantization (fastest, more accuracy loss)'
    )
    
    parser.add_argument(
        '--dynamic',
        action='store_true',
        help='Enable dynamic batch size (ONNX only)'
    )
    
    parser.add_argument(
        '--benchmark',
        action='store_true',
        help='Run benchmark after export'
    )
    
    args = parser.parse_args()
    
    try:
        # 옵티마이저 생성
        optimizer = ModelOptimizer(args.model)
        
        # 모델 변환
        output_path = None
        if args.format == 'onnx':
            output_path = optimizer.export_onnx(dynamic=args.dynamic)
        elif args.format == 'openvino':
            output_path = optimizer.export_openvino(half=args.fp16, int8=args.int8)
        elif args.format == 'tensorrt':
            output_path = optimizer.export_tensorrt(half=args.fp16)
        
        # 벤치마크
        if args.benchmark and output_path:
            optimizer.benchmark(format=args.format)
        
        print("\n✓ Optimization complete!")
        print(f"Output: {output_path}")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
