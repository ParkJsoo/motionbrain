"""현재 등록 영상의 도구 대리점과 물체 중심을 추적하는 오프라인 관측기.

세 번째 상태의 1은 차원 호환용 상수다. 턱폭·깊이·접촉·실제 파지점이나
이동 여유를 뜻하지 않는다. 표식·도구 가시성·물체 검사가 실패하면 거절한다.
"""
from __future__ import annotations
import copy
import hashlib
import json

import cv2
import numpy as np

SCHEMA = 'motionbrain.coarse_external_tool_proxy.v1'
MEANING = 'registered_2d_tool_proxy_and_object_center_not_jaw_opening_depth_contact_or_grasp'
PREFIX = 'coarse-external-tool-proxy-v1:'
STATE_FIELDS = ['objectMinusToolProxyXpx', 'objectMinusToolProxyYpx', 'constantCompatibility1']


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _require(ok, reason):
    if not ok: raise ValueError(reason)


def _image(raw):
    _require(isinstance(raw, bytes), 'coarse_image_bytes_required')
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    _require(image is not None and image.shape == (1080, 1920, 3), 'coarse_image_shape')
    return image


def _roi(image, rectangle):
    _require(isinstance(rectangle, list) and len(rectangle) == 4
             and all(type(v) is int for v in rectangle), 'coarse_roi')
    x, y, xx, yy = rectangle
    _require(0 <= x < xx <= image.shape[1] and 0 <= y < yy <= image.shape[0], 'coarse_roi')
    return image[y:yy, x:xx]


def _marker(image, roi, kind):
    patch = _roi(image, roi)
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    lo, hi = (([145, 30, 35], [175, 180, 210]) if kind == 'purple' else
              ([65, 25, 30], [105, 180, 190]))
    mask = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    count, _, stats, centers = cv2.connectedComponentsWithStats(mask)
    candidates = [(int(stats[i, cv2.CC_STAT_AREA]), centers[i]) for i in range(1, count)
                  if stats[i, cv2.CC_STAT_AREA] >= 80]
    _require(len(candidates) == 1, 'coarse_marker_missing_or_ambiguous')
    area, center = candidates[0]
    return area, [float(center[0]+roi[0]), float(center[1]+roi[1])]


def _object(image, roi):
    patch = _roi(image, roi)
    mask = cv2.inRange(cv2.cvtColor(patch, cv2.COLOR_BGR2HSV),
                       np.array([145, 110, 85], np.uint8), np.array([179, 255, 255], np.uint8))
    count, _, stats, centers = cv2.connectedComponentsWithStats(mask)
    candidates = []
    for i in range(1, count):
        x, y, w, h, area = map(int, stats[i])
        if 1000 <= area <= 20000 and 15 <= w <= 100 and 70 <= h <= 250 and 2 <= h/w <= 7:
            candidates.append(dict(center=[float(centers[i][0]+roi[0]), float(centers[i][1]+roi[1])],
                bbox=[x+roi[0], y+roi[1], w, h], area=area))
    _require(len(candidates) == 1, 'coarse_object_missing_or_ambiguous')
    return candidates[0]


def register(reference_bytes, *, tool_point, feature_roi, tool_patch_roi, marker_rois,
             object_roi=None, search_roi=None):
    image = _image(reference_bytes)
    _require(isinstance(tool_point, list) and len(tool_point) == 2
             and all(type(v) in (int, float) and np.isfinite(v) for v in tool_point), 'coarse_tool_point')
    _require(set(marker_rois) == {'purple', 'teal'}, 'coarse_marker_pair_required')
    object_roi = [500, 350, 1300, 1000] if object_roi is None else object_roi
    search_roi = [600, 60, 1350, 950] if search_roi is None else search_roi
    for roi in [feature_roi, tool_patch_roi, object_roi, search_roi, *marker_rois.values()]: _roi(image, roi)
    _require(tool_patch_roi[0] <= tool_point[0] < tool_patch_roi[2]
             and tool_patch_roi[1] <= tool_point[1] < tool_patch_roi[3], 'coarse_point_inside_tool_patch')
    markers = {kind: dict(zip(('area', 'center'), _marker(image, roi, kind))) for kind, roi in marker_rois.items()}
    registration = dict(schemaVersion=SCHEMA, referenceSha256=hashlib.sha256(reference_bytes).hexdigest(),
        imageShape=[1080, 1920], toolPoint=tool_point, featureRoi=feature_roi, toolPatchRoi=tool_patch_roi,
        markerRois=marker_rois, markerReference=markers, objectRoi=object_roi, searchRoi=search_roi,
        referenceObject=_object(image, object_roi), stateMeaning=MEANING,
        thresholds=dict(minimumMatches=15, minimumInliers=12, minimumInlierFraction=.2,
            maximumMedianResidualPx=2., minimumInlierSpanPx=[60., 25.], scaleRange=[.75, 1.25],
            maximumRotationDeg=65., markerCenterTolerancePx=12., markerAreaRatio=[.4, 2.5],
            toolSearchMarginPx=16, minimumToolNcc=.50, maximumToolCorrectionPx=18.,
            objectSizeRatio=[.65, 1.5]))
    return copy.deepcopy(registration)


class CoarseExternalObserver:
    def __init__(self, registration, reference_bytes):
        _require(registration.get('schemaVersion') == SCHEMA and registration.get('stateMeaning') == MEANING,
                 'coarse_registration_schema')
        _require(hashlib.sha256(reference_bytes).hexdigest() == registration.get('referenceSha256'),
                 'coarse_reference_changed')
        self.registration = copy.deepcopy(registration)
        self.binding = digest(self.registration)
        self.landmark_contract = PREFIX+self.binding
        self.reference = _image(reference_bytes)
        self.orb = cv2.ORB_create(nfeatures=3000, edgeThreshold=12, fastThreshold=8)
        mask = np.zeros(self.reference.shape[:2], np.uint8)
        x, y, xx, yy = self.registration['featureRoi']; mask[y:yy, x:xx] = 255
        self.points, self.descriptors = self.orb.detectAndCompute(self.reference, mask)
        _require(self.descriptors is not None and len(self.points) >= 30, 'coarse_reference_features')

    def observe(self, raw):
        _require(digest(self.registration) == self.binding, 'coarse_registration_changed')
        image = _image(raw); cfg = self.registration; limits = cfg['thresholds']
        object_result = _object(image, cfg['objectRoi'])
        for current, original in zip(object_result['bbox'][2:], cfg['referenceObject']['bbox'][2:]):
            _require(limits['objectSizeRatio'][0] <= current/original <= limits['objectSizeRatio'][1], 'coarse_object_shape_changed')
        mask = np.zeros(image.shape[:2], np.uint8)
        x, y, xx, yy = cfg['searchRoi']; mask[y:yy, x:xx] = 255
        points, descriptors = self.orb.detectAndCompute(image, mask)
        _require(descriptors is not None and len(points) >= 30, 'coarse_current_features')
        matches = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(self.descriptors, descriptors, k=2)
        matches = [m for pair in matches if len(pair) == 2 for m, n in [pair] if m.distance < .75*n.distance]
        _require(len(matches) >= limits['minimumMatches'], 'coarse_feature_matches')
        before = np.float32([self.points[m.queryIdx].pt for m in matches])
        after = np.float32([points[m.trainIdx].pt for m in matches])
        affine, selected = cv2.estimateAffinePartial2D(before, after, method=cv2.RANSAC, ransacReprojThreshold=2.)
        _require(affine is not None and selected is not None and np.isfinite(affine).all(), 'coarse_transform_missing')
        chosen = selected.ravel().astype(bool); inliers = int(chosen.sum())
        _require(inliers >= limits['minimumInliers'] and inliers/len(matches) >= limits['minimumInlierFraction'], 'coarse_transform_support')
        spans = np.ptp(before[chosen], axis=0)
        _require(all(spans >= np.array(limits['minimumInlierSpanPx'])), 'coarse_feature_span')
        predicted = cv2.transform(before[None, chosen], affine)[0]
        residual = float(np.median(np.linalg.norm(predicted-after[chosen], axis=1)))
        scale = float(np.hypot(affine[0, 0], affine[1, 0])); rotation = float(np.degrees(np.arctan2(affine[1, 0], affine[0, 0])))
        _require(residual <= limits['maximumMedianResidualPx'] and limits['scaleRange'][0] <= scale <= limits['scaleRange'][1]
                 and abs(rotation) <= limits['maximumRotationDeg'], 'coarse_transform_outside_registration')
        rectified = cv2.warpAffine(image, cv2.invertAffineTransform(affine), (1920, 1080))
        marker_quality = {}
        for kind, roi in cfg['markerRois'].items():
            area, center = _marker(rectified, roi, kind); ref = cfg['markerReference'][kind]
            distance = float(np.linalg.norm(np.array(center)-ref['center'])); ratio = area/ref['area']
            _require(distance <= limits['markerCenterTolerancePx'] and limits['markerAreaRatio'][0] <= ratio <= limits['markerAreaRatio'][1], 'coarse_marker_changed')
            marker_quality[kind] = dict(centerResidualPx=distance, areaRatio=ratio)
        x, y, xx, yy = cfg['toolPatchRoi']; margin = limits['toolSearchMarginPx']
        patch = _roi(self.reference, cfg['toolPatchRoi'])
        search = _roi(rectified, [x-margin, y-margin, xx+margin, yy+margin])
        scores = cv2.matchTemplate(cv2.cvtColor(search, cv2.COLOR_BGR2GRAY), cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY), cv2.TM_CCOEFF_NORMED)
        _, ncc, _, location = cv2.minMaxLoc(scores)
        correction = np.array(location, dtype=float)-margin
        _require(np.isfinite(ncc) and ncc >= limits['minimumToolNcc']
                 and np.linalg.norm(correction) <= limits['maximumToolCorrectionPx'], 'coarse_tool_not_visible')
        tip = cv2.transform(np.float32([[np.array(cfg['toolPoint'])+correction]]), affine)[0, 0].tolist()
        marker_axis = np.array(cfg['markerReference']['teal']['center'])-cfg['markerReference']['purple']['center']
        current_axis = affine[:, :2] @ marker_axis
        _require(float(np.linalg.norm(current_axis)) > 1., 'coarse_marker_axis_missing')
        tool_axis_deg = float(np.degrees(np.arctan2(current_axis[1], current_axis[0])))
        _require(0 <= tip[0] < 1920 and 0 <= tip[1] < 1080, 'coarse_tool_outside_image')
        state = [object_result['center'][0]-tip[0], object_result['center'][1]-tip[1], 1.]
        return dict(schemaVersion=SCHEMA, registrationSha256=self.binding,
            imageSha256=hashlib.sha256(raw).hexdigest(), referenceSha256=cfg['referenceSha256'],
            state=state, toolProxyPx=tip, object=object_result, stateMeaning=MEANING,
            toolAxisImageAngleDeg=tool_axis_deg,
            toolAxisMeaning='projected_purple_to_teal_marker_axis_not_physical_joint_angle',
            stateFields=STATE_FIELDS.copy(), landmarkContract=self.landmark_contract,
            quality=dict(matches=len(matches), inliers=inliers, medianResidualPx=residual,
                inlierSpanPx=spans.tolist(), scale=scale, rotationDeg=rotation, markers=marker_quality,
                toolNcc=float(ncc), toolCorrectionPx=correction.tolist()),
            contactVerified=False, graspVerified=False, depthVerified=False)


class CoarseExternalBank:
    """같은 대리점을 연결한 고정 기준 묶음. 관측 실패는 기록하고 합의 불일치는 거절한다.

    뒤 기준의 대리점은 앞 기준 중 하나가 해당 원본에서 측정한 점과 일치해야 한다.
    실행 중 기준 추가·학습·임계값 변경은 없고, 새 묶음은 새 landmark 계약이다.
    """
    def __init__(self, registrations, reference_bytes):
        _require(isinstance(registrations, list) and isinstance(reference_bytes, list)
                 and 1 <= len(registrations) == len(reference_bytes) <= 8, 'coarse_bank_references')
        self.references = tuple(reference_bytes)
        self.observers = []
        links = []
        seen = set()
        for registration, raw in zip(registrations, self.references):
            observer = CoarseExternalObserver(registration, raw)
            _require(observer.binding not in seen, 'coarse_bank_duplicate_reference')
            seen.add(observer.binding)
            _require(registration['thresholds'] == registrations[0]['thresholds'], 'coarse_bank_thresholds_changed')
            if self.observers:
                correspondences = []
                for previous in self.observers:
                    try: result = previous.observe(raw)
                    except ValueError: continue
                    distance = float(np.linalg.norm(np.array(result['toolProxyPx'])-registration['toolPoint']))
                    if distance <= .1:
                        correspondences.append(dict(registrationSha256=previous.binding,
                            imageSha256=result['imageSha256'], toolProxyPx=result['toolProxyPx'], residualPx=distance))
                _require(bool(correspondences), 'coarse_bank_unbound_proxy_correspondence')
                links.append(dict(childRegistrationSha256=observer.binding, parentCorrespondences=correspondences))
            self.observers.append(observer)
        self.registration = dict(schemaVersion='motionbrain.coarse_external_tool_proxy_bank.v1',
            registrations=copy.deepcopy(registrations), proxyCorrespondences=links,
            maximumDisagreementPx=10., selection='highest_tool_ncc_after_all_accepted_proxies_agree',
            stateMeaning=MEANING)
        self.binding = digest(self.registration)
        self.landmark_contract = PREFIX+self.binding

    def observe(self, raw):
        _require(digest(self.registration) == self.binding, 'coarse_bank_registration_changed')
        accepted, rejected = [], []
        for index, (observer, reference) in enumerate(zip(self.observers, self.references)):
            _require(hashlib.sha256(reference).hexdigest() == observer.registration['referenceSha256']
                     and observer.registration == self.registration['registrations'][index], 'coarse_bank_reference_changed')
            try:
                result = observer.observe(raw)
                accepted.append(result)
            except ValueError as exc:
                rejected.append(dict(registrationSha256=observer.binding, reason=str(exc)))
        _require(bool(accepted), 'coarse_bank_no_supported_reference')
        disagreements = [float(np.linalg.norm(np.array(a['toolProxyPx'])-b['toolProxyPx']))
                         for i, a in enumerate(accepted) for b in accepted[i+1:]]
        _require(not disagreements or max(disagreements) <= self.registration['maximumDisagreementPx'],
                 'coarse_bank_proxy_disagreement')
        selected = max(accepted, key=lambda row: row['quality']['toolNcc'])
        result = copy.deepcopy(selected)
        result.update(registrationSha256=self.binding, landmarkContract=self.landmark_contract,
            bankSchemaVersion=self.registration['schemaVersion'],
            selectedReferenceRegistrationSha256=selected['registrationSha256'],
            referenceSha256s=[observer.registration['referenceSha256'] for observer in self.observers],
            acceptedReferences=[dict(registrationSha256=row['registrationSha256'], toolProxyPx=row['toolProxyPx'],
                toolNcc=row['quality']['toolNcc'], toolAxisImageAngleDeg=row['toolAxisImageAngleDeg']) for row in accepted],
            rejectedReferences=rejected, maximumProxyDisagreementPx=max(disagreements, default=0.))
        result['maximumAxisDisagreementDeg'] = max((abs((a['toolAxisImageAngleDeg']-b['toolAxisImageAngleDeg']+180.) % 360.-180.)
            for i, a in enumerate(accepted) for b in accepted[i+1:]), default=0.)
        return result
