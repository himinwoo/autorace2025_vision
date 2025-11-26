# 허프라인 검출 개선 사항

## 개선 날짜
2025년 11월 26일

## 문제점 분석

### 기존 코드의 문제점

1. **잘못된 입력 이미지 사용**
   - 허프라인을 `edges_dilated`에서 검출
   - 이는 이미 bilateral filter, CLAHE, adaptive threshold, morphology 등의 전처리가 완료된 에지
   - 과도한 전처리로 인해 정지선의 일부가 손실될 수 있음

2. **처리 순서 문제**
   - 허프라인 검출이 히스토그램 기반 검출 **이후**에 수행됨
   - 히스토그램 방식을 위한 전처리가 허프라인 검출에도 영향을 미침
   - 두 방법은 독립적으로 수행되어야 하는데 순차적으로 처리됨

3. **각도 범위 제한**
   - 대각선 각도만 검출 (10~70도, 110~160도)
   - 수평에 가까운 정지선 (0~10도)이 누락될 수 있음
   - 실제 정지선은 대부분 수평 또는 약간의 경사만 있음

4. **파라미터 하드코딩**
   ```python
   lines = cv2.HoughLinesP(edges_dilated, 1, np.pi/180, threshold=60, minLineLength=30, maxLineGap=20)
   if (10 < angle < 70) or (110 < angle < 170):
       cv2.line(hough_mask, (x1, y1), (x2, y2), 1, 8)
   ```
   - 모든 파라미터가 하드코딩되어 있어 조정이 어려움
   - 라인 두께가 1로 너무 얇음

5. **중복 전처리**
   - 히스토그램용 전처리와 허프라인용 전처리가 분리되지 않음
   - 불필요한 계산 반복

## 개선 사항

### 1. 독립적인 허프라인 검출 함수 추가

```python
def _detect_hough_lines(self, gray_img):
    """
    허프라인 변환을 이용한 정지선 검출
    원본 그레이스케일 이미지에서 직접 검출하여 전처리 손실 최소화
    """
```

**핵심 개선:**
- 원본 그레이스케일 이미지를 입력으로 받음
- 최소한의 전처리만 수행 (가우시안 블러)
- 히스토그램 처리와 완전히 독립적

### 2. 처리 순서 재구성

```python
def detect_white_stopline(self, img):
    stopline_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # ===== 허프라인 검출 (독립적으로 먼저 수행) =====
    hough_mask = self._detect_hough_lines(stopline_gray)
    
    # ===== 히스토그램 기반 검출 =====
    # ... (기존 처리)
    
    # ===== OR 연산으로 결과 합치기 =====
    stopline_bin = cv2.bitwise_or(histo_bin, hough_mask)
```

**핵심 개선:**
- 허프라인 검출을 최우선으로 수행
- 히스토그램 처리와 병렬적 구조
- 두 결과를 OR 연산으로 결합

### 3. 각도 필터링 개선

```python
# 각도 정규화 (-180 ~ 180)
if angle < 0:
    angle += 180

# 수평선에 가까운 라인만 선택 (0도, 180도 근처)
is_near_horizontal = (
    angle <= self.hough_angle_tolerance or 
    angle >= (180 - self.hough_angle_tolerance)
)
```

**핵심 개선:**
- 수평선 중심의 각도 필터링 (기본값: ±25도)
- 정지선의 실제 특성 반영
- 파라미터로 조정 가능

### 4. 라인 길이 필터링 추가

```python
line_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# 충분히 긴 라인만 사용 (정지선은 이미지 폭의 상당 부분 차지)
if line_length > gray_img.shape[1] * 0.15:  # 이미지 폭의 15% 이상
    cv2.line(hough_mask, (x1, y1), (x2, y2), 255, thickness=5)
```

**핵심 개선:**
- 이미지 폭 대비 상대적 길이 검사 (15% 이상)
- 짧은 노이즈 라인 제거
- 두꺼운 선 (thickness=5)으로 그려서 히스토그램과 결합 시 효과적

### 5. 파라미터화

새로 추가된 ROS 파라미터:
- `hough_threshold` (int, default: 50): 허프라인 검출 임계값
- `hough_min_line_length` (int, default: 40): 허프라인 최소 길이
- `hough_max_line_gap` (int, default: 10): 허프라인 최대 간격
- `hough_angle_tolerance` (float, default: 25.0): 수평선 기준 각도 허용 범위
- `hough_use_morphology` (bool, default: true): 모폴로지 연산 적용 여부

**핵심 개선:**
- 모든 파라미터를 launch 파일에서 조정 가능
- 실시간 튜닝 용이
- 다양한 환경에 대한 적응성 향상

### 6. 선택적 모폴로지 연산

```python
if self.hough_use_morphology and np.any(hough_mask > 0):
    # 수평 방향 확장 (정지선 연속성 보장)
    horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 3))
    hough_mask = cv2.morphologyEx(hough_mask, cv2.MORPH_CLOSE, horizontal_kernel)
    
    # 수직 방향 확장 (정지선 두께 보장)
    vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 7))
    hough_mask = cv2.dilate(hough_mask, vertical_kernel, iterations=1)
```

**핵심 개선:**
- 수평/수직 커널을 분리하여 정지선 특성에 맞게 처리
- 선택적 적용으로 유연성 확보
- 라인의 연속성과 두께 보장

## 알고리즘 비교

### 기존 알고리즘
```
그레이스케일 변환
  → 과포화 마스킹
  → Bilateral 필터
  → CLAHE
  → Otsu + Adaptive threshold
  → Canny 에지
  → 에지 팽창 (edges_dilated)
  → 허프라인 검출 (edges_dilated에서)  ⚠️ 문제!
  → 대각선 각도만 필터링 (10~70도)
```

### 개선된 알고리즘
```
[허프라인 경로 - 독립적]
그레이스케일 변환
  → 가우시안 블러 (경미)
  → Canny 에지 (원본에서)
  → 허프라인 검출 ✓
  → 수평선 중심 필터링 (±25도)
  → 길이 필터링 (폭의 15% 이상)
  → 선택적 모폴로지

[히스토그램 경로 - 독립적]
그레이스케일 변환
  → 과포화 마스킹
  → Bilateral 필터
  → CLAHE
  → Otsu + Adaptive threshold
  → Canny 에지 + 팽창
  → 형태학적 필터링
  → 원형도 필터링

[결합]
허프라인 결과 OR 히스토그램 결과 ✓
```

## 예상 효과

1. **검출 정확도 향상**
   - 원본 이미지에서 직접 검출하여 정보 손실 최소화
   - 수평 정지선 검출률 향상

2. **처리 안정성 향상**
   - 독립적인 두 가지 방법의 병렬 처리
   - 한 방법이 실패해도 다른 방법으로 보완

3. **튜닝 용이성 증가**
   - 모든 파라미터를 launch 파일에서 조정 가능
   - 실험 및 최적화 시간 단축

4. **다양한 환경 대응**
   - 조명 변화, 노면 상태, 정지선 각도 등에 유연하게 대응
   - 파라미터 조정으로 다양한 시나리오 대응

## 사용 방법

### 기본 사용
```bash
roslaunch stopline_pkg stopline_detection.launch
```

### 파라미터 조정 예시

밝은 환경에서 허프라인 민감도 증가:
```xml
<param name="hough_threshold" value="40" />
<param name="hough_angle_tolerance" value="20.0" />
```

어두운 환경에서 더 관대한 검출:
```xml
<param name="hough_threshold" value="30" />
<param name="hough_min_line_length" value="30" />
<param name="hough_angle_tolerance" value="30.0" />
```

기울어진 정지선이 많은 코스:
```xml
<param name="hough_angle_tolerance" value="35.0" />
```

## 결론

이번 개선으로 허프라인 검출이:
1. **독립적이고 병렬적**으로 처리됨
2. **원본에 가까운 이미지**에서 검출하여 정보 손실 최소화
3. **정지선 특성에 최적화**된 필터링 적용
4. **완전히 파라미터화**되어 조정 용이

이를 통해 정지선 감지의 **정확도, 안정성, 유연성**이 모두 향상될 것으로 기대됩니다.
