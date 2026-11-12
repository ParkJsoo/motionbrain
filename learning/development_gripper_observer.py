"""집게 영상의 오프라인 개발 관측기. 실시간 등록·학습·구동 권한은 없다.

색 영역과 보이는 턱끝을 각각 관측한다. 가림·잘림·모호한 일치는 빈 값으로
남기며, 기존 등록 관측기의 계약이나 적격 판정을 대신하지 않는다.
"""
from __future__ import annotations

import copy
import hashlib

import cv2
import numpy as np

from learning.batch_observation import digest, number, require

SCHEMA = 'motionbrain.development_gripper_observer.v1'
CONTEXT = {'viewId', 'poseRegime', 'object'}
JAWS = ('jawLeft', 'jawRight')


def default_parameters():
    return dict(imageShape=[240, 320], hsvLower=[145, 90, 100], hsvUpper=[179, 255, 255],
        colorRows=[75, 240], minimumColorAreaPx=100, maximumColorAreaPx=20000,
        frameEdgeMarginPx=4, minimumNcc=.90, minimumPeakGap=.04,
        peakExclusionRadius=3, minimumTemplateStd=1.5, cornerColorRadiusPx=3,
        cornerSupportRadiusPx=4, minimumCornerSupportNcc=.90,
        maximumReferenceDisagreementPx=4.,
        jawSearchRois=dict(jawLeft=[65, 65, 155, 160], jawRight=[165, 65, 255, 160]))


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def decode(raw, shape):
    require(isinstance(raw, bytes) and 0 < len(raw) <= 2_000_000, 'development_image_bytes')
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    require(image is not None and list(image.shape[:2]) == shape, 'development_image_shape')
    return image


def _roi(value, shape):
    require(isinstance(value, list) and len(value) == 4 and all(type(v) is int for v in value),
            'development_roi')
    x0, y0, x1, y1 = value
    require(0 <= x0 < x1 <= shape[1] and 0 <= y0 < y1 <= shape[0]
            and min(x1-x0, y1-y0) >= 8, 'development_roi')
    return x0, y0, x1, y1


class DevelopmentGripperObserver:
    def __init__(self, context, references, *, parameters=None):
        require(isinstance(context, dict) and set(context) == CONTEXT and
                all(isinstance(v, str) and v for v in context.values()), 'development_context')
        self.context = copy.deepcopy(context)
        self.parameters = copy.deepcopy(default_parameters() if parameters is None else parameters)
        # 다른 자세에 대한 자동 확장·임계값 완화를 금지한 명시적 개발 정책이다.
        require(self.parameters == default_parameters(), 'development_fixed_policy_changed')
        require(isinstance(references, list) and 1 <= len(references) <= 8,
                'development_references_required')
        self.references, self.templates = [], []
        for reference in references:
            require(isinstance(reference, dict) and set(reference) ==
                    {'id', 'imageBytes', 'imageSha256', 'points', 'rois'}, 'development_reference_fields')
            require(isinstance(reference['id'], str) and reference['id'], 'development_reference_id')
            require(sha(reference['imageBytes']) == reference['imageSha256'], 'development_reference_hash')
            image = decode(reference['imageBytes'], self.parameters['imageShape'])
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            require(set(reference['points']) == set(reference['rois']) == set(JAWS),
                    'development_reference_jaws')
            item = {k: copy.deepcopy(v) for k, v in reference.items() if k != 'imageBytes'}
            self.references.append(item)
            for name in JAWS:
                x0, y0, x1, y1 = _roi(reference['rois'][name], self.parameters['imageShape'])
                point = reference['points'][name]
                require(isinstance(point, list) and len(point) == 2 and all(number(v) for v in point)
                        and x0 <= point[0] < x1 and y0 <= point[1] < y1, 'development_seed_point')
                radius = self.parameters['cornerSupportRadiusPx']
                require(all(type(v) is int for v in point) and
                        x0+radius <= point[0] < x1-radius and y0+radius <= point[1] < y1-radius,
                        'development_corner_support_bounds')
                template = gray[y0:y1, x0:x1].copy()
                require(float(template.std()) >= self.parameters['minimumTemplateStd'],
                        'development_template_texture')
                template.flags.writeable = False
                self.templates.append((reference['id'], name, template, [point[0]-x0, point[1]-y0]))
        require(len({r['id'] for r in self.references}) == len(self.references), 'development_duplicate_reference')
        self.binding = digest(dict(context=self.context, parameters=self.parameters, references=self.references))

    def _color(self, image):
        p = self.parameters
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(p['hsvLower']), np.array(p['hsvUpper']))
        first, last = p['colorRows']
        mask[:first] = 0; mask[last:] = 0
        count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
        candidates = [stats[i] for i in range(1, count)
                      if p['minimumColorAreaPx'] <= stats[i, cv2.CC_STAT_AREA]]
        if len(candidates) != 1:
            return dict(status='unavailable', reason='color_missing' if not candidates else 'color_ambiguous',
                        box=None, center=None, candidateCount=len(candidates)), mask
        x, y, width, height, area = map(int, candidates[0])
        if area > p['maximumColorAreaPx']:
            return dict(status='unavailable', reason='color_area_exceeded', box=None,
                        center=None, candidateCount=1), mask
        box = [x, y, x+width, y+height]
        margin = p['frameEdgeMarginPx']
        clipped = (x <= margin or y <= first+margin or x+width >= image.shape[1]-margin
                   or y+height >= min(last, image.shape[0])-margin)
        if clipped:
            return dict(status='unavailable', reason='color_at_observation_boundary', box=None,
                        center=None, diagnosticCandidateBox=box, candidateCount=1), mask
        return dict(status='visible_development', reason=None, box=box,
                    center=[x+width/2, y+height/2], areaPx=area, candidateCount=1), mask

    def _jaw(self, gray, mask, name):
        p = self.parameters
        sx0, sy0, sx1, sy1 = p['jawSearchRois'][name]
        candidates, diagnostics = [], []
        for reference_id, landmark, template, offset in self.templates:
            if landmark != name:
                continue
            search = gray[sy0:sy1, sx0:sx1]
            scores = cv2.matchTemplate(search, template, cv2.TM_CCOEFF_NORMED)
            require(np.isfinite(scores).all(), 'development_nonfinite_match')
            _, peak, _, (x, y) = cv2.minMaxLoc(scores)
            others = scores.copy(); radius = p['peakExclusionRadius']
            others[max(0, y-radius):y+radius+1, max(0, x-radius):x+radius+1] = -2
            gap = float(peak-others.max())
            point = [sx0+x+offset[0], sy0+y+offset[1]]
            diagnostic = dict(referenceId=reference_id, ncc=float(peak), peakGap=gap,
                              diagnosticPoint=point)
            diagnostics.append(diagnostic)
            boundary = x in (0, scores.shape[1]-1) or y in (0, scores.shape[0]-1)
            if boundary or peak < p['minimumNcc'] or gap < p['minimumPeakGap']:
                continue
            px, py = (int(round(v)) for v in point)
            # 패치 주변만 맞고 실제 끝점이 가려진 경우 위치를 복원하지 않는다.
            radius = p['cornerSupportRadiusPx']
            tx, ty = offset
            seed_corner = template[ty-radius:ty+radius+1, tx-radius:tx+radius+1]
            current_corner = gray[py-radius:py+radius+1, px-radius:px+radius+1]
            if min(float(seed_corner.std()), float(current_corner.std())) < p['minimumTemplateStd']:
                diagnostic['rejectedReason'] = 'corner_local_support_missing'
                continue
            local_ncc = float(cv2.matchTemplate(current_corner, seed_corner, cv2.TM_CCOEFF_NORMED)[0, 0])
            diagnostic['localCornerNcc'] = local_ncc
            if not np.isfinite(local_ncc) or local_ncc < p['minimumCornerSupportNcc']:
                diagnostic['rejectedReason'] = 'corner_local_support_mismatch'
                continue
            radius = p['cornerColorRadiusPx']
            if np.any(mask[max(0, py-radius):py+radius+1, max(0, px-radius):px+radius+1]):
                diagnostic['rejectedReason'] = 'corner_color_occlusion_or_contact_appearance'
                continue
            candidates.append((float(peak), point, reference_id))
        if not candidates:
            return dict(status='unavailable', point=None, reason='corner_no_unambiguous_visible_match',
                        diagnostics=diagnostics)
        _, point, reference_id = max(candidates)
        if any(np.linalg.norm(np.asarray(other[1])-point) > p['maximumReferenceDisagreementPx']
               for other in candidates):
            return dict(status='unavailable', point=None, reason='corner_reference_disagreement',
                        diagnostics=diagnostics)
        return dict(status='visible_development', point=point, referenceId=reference_id,
                    reason=None, diagnostics=diagnostics)

    def observe(self, raw, frame_id, context, *, image_sha256):
        require(context == self.context, 'development_context_changed')
        require(isinstance(frame_id, str) and frame_id, 'development_frame_id')
        require(digest(dict(context=self.context, parameters=self.parameters, references=self.references)) == self.binding,
                'development_configuration_changed')
        require(sha(raw) == image_sha256, 'development_image_hash')
        image = decode(raw, self.parameters['imageShape'])
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        color, mask = self._color(image)
        jaws = {name: self._jaw(gray, mask, name) for name in JAWS}
        left, right = [jaws[name]['point'] for name in JAWS]
        if left is not None and right is not None and (left[0] >= right[0] or abs(left[1]-right[1]) > 10):
            for name in JAWS:
                jaws[name].update(status='unavailable', point=None, reason='corner_pair_geometry_changed')
        complete = color['box'] is not None and all(jaws[name]['point'] is not None for name in JAWS)
        features = None
        if complete:
            left, right = [jaws[name]['point'] for name in JAWS]
            mid = [(left[i]+right[i])/2 for i in (0, 1)]
            features = dict(objectVisibleBoxCenterPx=color['center'], jawMidpointPx=mid,
                jawHorizontalSpanPx=right[0]-left[0],
                objectMinusJawMidpointPx=[color['center'][i]-mid[i] for i in (0, 1)])
        return dict(schemaVersion=SCHEMA, frameId=frame_id, imageSha256=image_sha256,
            decodedRgbSha256=sha(cv2.cvtColor(image, cv2.COLOR_BGR2RGB).tobytes()),
            configurationSha256=self.binding, context=copy.deepcopy(self.context),
            status='complete_development' if complete else 'partial_development',
            object=color, jaws=jaws, features=features,
            referenceImageSelfReplay=image_sha256 in {r['imageSha256'] for r in self.references},
            inputFrameOnly=True, trainingEligible=False, controlEligible=False,
            liveAdmissionVerified=False, measurementAccuracyVerified=False,
            semantics='visible_color_box_and_visible_corner_pixels_not_grasp_depth_contact_force_or_slip')
