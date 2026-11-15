"""현재 보이는 두 표식에서 직접 계산하는 가상 영상 특징점.

가상점은 표식 좌표계의 고정 선형 결합이다. 가려진 턱 끝을 복원하거나 실제
파지점·접촉·깊이·충돌 여유를 측정하지 않는다. 검은 픽셀 검사는 주변 시각적
지지의 품질 조건이며 그 픽셀이 실제 턱이라는 보장도 아니다.
"""
from __future__ import annotations
import copy
import hashlib

import cv2
import numpy as np

from learning.coarse_external_observer import _image, _object, _roi, _require, digest

SCHEMA = 'motionbrain.marker_derived_virtual_point.v1'
PREFIX = 'marker-derived-virtual-point-v1:'
STATE_FIELDS = ['objectMinusVirtualPointXpx', 'objectMinusVirtualPointYpx', 'constantCompatibility1']
MEANING = 'fixed_marker_basis_virtual_image_point_not_actual_fingertip_depth_contact_grasp_or_clearance'


def detector_policy():
    return dict(workingRoi=[500, 100, 1400, 950], objectRoi=[500, 350, 1300, 1000],
        purpleHsvLow=[145, 35, 55], purpleHsvHigh=[175, 180, 210],
        tealHsvLow=[65, 35, 55], tealHsvHigh=[105, 180, 190],
        purpleArea=[80, 800], tealArea=[80, 600], aspectRatio=[.35, 3.],
        minimumFillFraction=.4, minimumSolidity=.7, openingKernelSize=3,
        pairDistancePx=[15., 60.], uniquePairRequired=True,
        blackWindowRadiusPx=22, blackMaximumValue=85, minimumBlackOccupancyFraction=.15,
        blackPresenceMeaning='local_dark_pixel_support_not_actual_tip_or_collision_validation',
        objectDetector='pink_unique_upright_component_v1',
        objectHsvLow=[145, 110, 85], objectHsvHigh=[179, 255, 255],
        objectArea=[1000, 20000], objectWidth=[15, 100], objectHeight=[70, 250], objectAspectRatio=[2., 7.])


def _candidates(image, kind, policy):
    roi = policy['workingRoi']; patch = _roi(image, roi)
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.uint8(policy[kind+'HsvLow']), np.uint8(policy[kind+'HsvHigh']))
    size = policy['openingKernelSize']
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((size, size), np.uint8))
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
    candidates = []
    for index in range(1, count):
        x, y, width, height, area = map(int, stats[index])
        fill = area/(width*height)
        if not (policy[kind+'Area'][0] <= area <= policy[kind+'Area'][1]
                and policy['aspectRatio'][0] <= width/height <= policy['aspectRatio'][1]
                and fill >= policy['minimumFillFraction']): continue
        contours, _ = cv2.findContours(np.uint8(labels == index), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour = max(contours, key=cv2.contourArea)
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        solidity = cv2.contourArea(contour)/hull_area if hull_area else 0.
        if solidity < policy['minimumSolidity']: continue
        candidates.append(dict(center=(centers[index]+np.array(roi[:2])).tolist(), area=area,
            bbox=[x+roi[0], y+roi[1], width, height], fillFraction=fill, solidity=float(solidity)))
    return candidates


def _pair(image, policy):
    purple = _candidates(image, 'purple', policy); teal = _candidates(image, 'teal', policy)
    pairs = []
    for a in purple:
        for b in teal:
            distance = float(np.linalg.norm(np.array(b['center'])-a['center']))
            if policy['pairDistancePx'][0] <= distance <= policy['pairDistancePx'][1]:
                pairs.append(dict(purple=a, teal=b, distancePx=distance))
    _require(len(pairs) == 1, 'virtual_marker_pair_missing_or_ambiguous')
    return pairs[0], dict(purpleCandidates=len(purple), tealCandidates=len(teal), eligiblePairs=len(pairs))


def _basis(pair):
    origin = np.array(pair['purple']['center'])
    axis = np.array(pair['teal']['center'])-origin
    return origin, np.column_stack((axis, [-axis[1], axis[0]]))


def register_virtual(seed_bytes, *, virtual_point):
    image = _image(seed_bytes); policy = detector_policy()
    _require(isinstance(virtual_point, list) and len(virtual_point) == 2
             and all(type(v) in (int, float) and np.isfinite(v) for v in virtual_point)
             and 0 <= virtual_point[0] < 1920 and 0 <= virtual_point[1] < 1080, 'virtual_seed_point')
    pair, _ = _pair(image, policy); origin, basis = _basis(pair)
    coefficients = np.linalg.solve(basis, np.array(virtual_point)-origin)
    return dict(schemaVersion=SCHEMA, seedSha256=hashlib.sha256(seed_bytes).hexdigest(), imageShape=[1080, 1920],
        seedVirtualPoint=copy.deepcopy(virtual_point), seedMarkerPair=pair, basisCoefficients=coefficients.tolist(),
        detectorPolicy=policy, stateMeaning=MEANING, referenceObject=_object(image, policy['objectRoi']))


class VirtualMarkerObserver:
    def __init__(self, registration, seed_bytes):
        _require(isinstance(registration, dict) and registration.get('schemaVersion') == SCHEMA
                 and registration.get('stateMeaning') == MEANING
                 and registration.get('detectorPolicy') == detector_policy(), 'virtual_registration_contract')
        _require(hashlib.sha256(seed_bytes).hexdigest() == registration.get('seedSha256'), 'virtual_seed_changed')
        _require(register_virtual(seed_bytes, virtual_point=registration['seedVirtualPoint']) == registration,
                 'virtual_seed_registration_mismatch')
        self.registration = copy.deepcopy(registration); self.seed_bytes = seed_bytes
        self.binding = digest(self.registration); self.landmark_contract = PREFIX+self.binding

    def observe(self, raw):
        _require(digest(self.registration) == self.binding
                 and hashlib.sha256(self.seed_bytes).hexdigest() == self.registration['seedSha256'],
                 'virtual_registration_changed')
        image = _image(raw); policy = self.registration['detectorPolicy']
        pair, counts = _pair(image, policy); origin, basis = _basis(pair)
        point = origin+basis@np.array(self.registration['basisCoefficients'])
        radius = policy['blackWindowRadiusPx']; x, y = np.rint(point).astype(int)
        _require(radius <= x < 1920-radius and radius <= y < 1080-radius, 'virtual_point_outside_image')
        patch = image[y-radius:y+radius+1, x-radius:x+radius+1]
        black = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)[:, :, 2] < policy['blackMaximumValue']
        black_fraction = float(black.mean())
        _require(black_fraction >= policy['minimumBlackOccupancyFraction'], 'virtual_local_dark_support_missing')
        target = _object(image, policy['objectRoi'])
        state = [target['center'][0]-float(point[0]), target['center'][1]-float(point[1]), 1.]
        axis = basis[:, 0]; heading = float(np.degrees(np.arctan2(axis[1], axis[0])))
        return dict(schemaVersion=SCHEMA, registrationSha256=self.binding, landmarkContract=self.landmark_contract,
            imageSha256=hashlib.sha256(raw).hexdigest(), referenceSha256=self.registration['seedSha256'],
            state=state, stateFields=STATE_FIELDS.copy(), stateMeaning=MEANING,
            virtualProxyPx=point.tolist(), toolProxyPx=point.tolist(), object=target,
            observedMarkerPair=pair, toolAxisImageAngleDeg=heading,
            toolAxisMeaning='direct_observed_purple_to_teal_image_axis_not_physical_joint_angle',
            quality=dict(**counts, markerDistancePx=pair['distancePx'], blackOccupancyFraction=black_fraction,
                blackPresenceMeaning=policy['blackPresenceMeaning']),
            actualFingertipMeasured=False, occludedTipReconstructed=False,
            contactVerified=False, graspVerified=False, depthVerified=False, clearanceVerified=False)
