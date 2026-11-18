"""기울어진 단일 분홍 물체의 별도 영상 관측. 지지·접촉·파지는 판정하지 않는다.

기존 직립 물체/모델 계약을 대체하지 않는다. 중심은 관측된 색 픽셀의 무게중심이며
회전 박스는 형상 선별에만 쓴다. 가림 아래 물체 크기나 탁상 접촉을 복원하지 않는다.
"""
from __future__ import annotations
import hashlib

import cv2
import numpy as np

from learning.coarse_external_observer import _image, _roi, _require, digest


def policy():
    return dict(schemaVersion='motionbrain.rotated_pink_component.v1',
        objectRoi=[500, 350, 1300, 1000], hsvLow=[145, 110, 85], hsvHigh=[179, 255, 255],
        area=[1000, 20000], shortSidePx=[15., 100.], longSidePx=[70., 250.],
        longShortRatio=[2., 7.], uniqueComponentRequired=True,
        centerMeaning='observed_pixel_centroid_not_contact_or_support')


def observe(raw):
    cfg = policy(); image = _image(raw); roi = cfg['objectRoi']
    patch = _roi(image, roi)
    mask = cv2.inRange(cv2.cvtColor(patch, cv2.COLOR_BGR2HSV),
                       np.uint8(cfg['hsvLow']), np.uint8(cfg['hsvHigh']))
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
    candidates = []
    for i in range(1, count):
        x, y, w, h, area = map(int, stats[i])
        if not cfg['area'][0] <= area <= cfg['area'][1]:
            continue
        ys, xs = np.where(labels == i)
        rectangle = cv2.minAreaRect(np.column_stack((xs, ys)).astype(np.float32))
        short, long = sorted(rectangle[1])
        if not (cfg['shortSidePx'][0] <= short <= cfg['shortSidePx'][1]
                and cfg['longSidePx'][0] <= long <= cfg['longSidePx'][1]
                and cfg['longShortRatio'][0] <= long/short <= cfg['longShortRatio'][1]):
            continue
        candidates.append(dict(center=(centers[i]+roi[:2]).tolist(),
            bbox=[x+roi[0], y+roi[1], w, h], area=area,
            rotatedShortLongPx=[float(short), float(long)],
            rotatedBoxPx=(cv2.boxPoints(rectangle)+roi[:2]).tolist()))
    _require(len(candidates) == 1, 'rotated_object_missing_or_ambiguous')
    return dict(schemaVersion=cfg['schemaVersion'], policySha256=digest(cfg),
        imageSha256=hashlib.sha256(raw).hexdigest(), object=candidates[0],
        centerMeaning=cfg['centerMeaning'], contactVerified=False,
        supportVerified=False, graspVerified=False, depthVerified=False,
        existingModelInputAdmissionVerified=False)
