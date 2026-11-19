"""고정 집게 카메라의 열린 집게 시작 장면용 오프라인 좌표 판독기.

반환값은 영상 좌표이며 접근·닫기 허가나 깊이 판정에 사용하지 않는다.
"""
import cv2
import numpy as np

from learning.near_gripper_tip_observer import _object, _tip, observer_policy


def detect_start_landmarks(jpeg):
    """320×240 영상에서 왼턱·오른턱의 보이는 아래 끝과 분홍 영역 중심을 반환한다."""
    if not isinstance(jpeg, bytes) or not jpeg:
        raise ValueError('near_jpeg_required')
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape != (240, 320, 3):
        raise ValueError('near_qvga_image_required')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    policy = observer_policy(version=3)
    # 가로 배경 경계를 제거한 뒤에도 기존 양턱 형상·단일 후보 검사를 적용한다.
    dark = np.uint8(hsv[:, :, 2] < policy['blackMaximumValue']) * 255
    vertical = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((17, 1), np.uint8))
    finger_image = hsv.copy()
    finger_image[:, :, 2] = 255 - vertical
    # 열린 시작 자세의 손가락 창. 아래 경계에 닿은 후보는 _tip에서 보류한다.
    policy['leftRoi'] = [20, 35, 160, 140]
    policy['rightRoi'] = [160, 35, 300, 140]
    left, right = [_tip(finger_image, policy[side], policy)['point']
                   for side in ('leftRoi', 'rightRoi')]
    if (right[0] - left[0] < policy['minimumGapPx'] or
            abs(left[1] - right[1]) > policy['maximumTipYDifferencePx']):
        raise ValueError('near_visible_tips_not_separated')
    # 시작 시 멀리 보이는 물체도 전체 높이에서 탐색한다.
    policy['objectRoi'] = [0, 0, 320, 240]
    obj, _ = _object(hsv, policy)
    x, y, w, h = obj['bbox']
    if x == 0 or y == 0 or x+w == 320 or y+h == 240:
        raise ValueError('near_object_clipped')
    return [left, right, obj['center']]
