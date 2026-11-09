"""M4만 움직이는 고정 집게의 투영점: 기존 광류·강체·고정 배경 검사를 재사용한다.

턱 개폐나 다른 관절이 움직일 때 적용하지 않는다. 기준점의 강체 가정은
구도 선언이며 실제 깊이·접촉·관절각 보정 또는 제어 권한을 만들지 않는다.
"""
import copy
import cv2
import numpy as np
from learning import arm_motion_observation as motion
from learning import registered_scene_landmarks as scene

SCHEMA = 'motionbrain.registered_rigid_scene_landmarks.v1'
METHOD = 'registered_scene_rigid_points'
PREFIX = 'registered-scene-rigid-points-v1:'


def register(base_registration, reference, moving_polygon, background_polygons):
    base = scene.RegisteredSceneLandmarks(base_registration, reference)
    parameters = dict(baseRegistration=copy.deepcopy(base_registration),
        movingPolygon=copy.deepcopy(moving_polygon), backgroundPolygons=copy.deepcopy(background_polygons),
        qualityParameters=motion._configuration(None), motionScope='M4_only_other_joints_and_gripper_fixed')
    result = dict(schemaVersion=SCHEMA, parameters=parameters, parametersSha256=scene._digest(parameters),
        referenceImageSha256=scene._sha(reference), imageShape=base.registration['imageShape'],
        points=copy.deepcopy(base.registration['points']), sceneContext=copy.deepcopy(base.registration['sceneContext']),
        trainingEligible=False, controlEligible=False)
    result['registrationSha256'] = scene._digest(result)
    RegisteredRigidSceneLandmarks(result, reference).observe(reference, 'rigid-registration-self-check')
    return result


class RegisteredRigidSceneLandmarks:
    def __init__(self, registration, reference_image):
        self.registration = copy.deepcopy(registration)
        self.reference_image = reference_image
        self.verify()

    @property
    def landmark_contract(self):
        return PREFIX+self.registration['registrationSha256']

    def verify(self):
        r = self.registration
        scene._require(r.get('schemaVersion') == SCHEMA and r.get('trainingEligible') is False
            and r.get('controlEligible') is False
            and r.get('registrationSha256') == scene._digest({k:v for k,v in r.items() if k!='registrationSha256'})
            and r.get('parametersSha256') == scene._digest(r['parameters'])
            and r.get('referenceImageSha256') == scene._sha(self.reference_image), 'rigid_scene_contract_changed')
        p = r['parameters']
        scene._require(p['qualityParameters'] == motion._configuration(None)
            and p['motionScope'] == 'M4_only_other_joints_and_gripper_fixed', 'rigid_scene_scope_changed')
        base = scene.RegisteredSceneLandmarks(p['baseRegistration'], self.reference_image)
        scene._require(all(r[k] == base.registration[k] for k in ('points','sceneContext','imageShape')),
                       'rigid_scene_base_changed')

    def observe(self, image_bytes, frame_id):
        self.verify()
        scene._require(isinstance(frame_id,str) and 0<len(frame_id)<=160,'scene_frame_id_required')
        r = self.registration;p = r['parameters'];base = p['baseRegistration']
        first, _ = scene._image(self.reference_image,r['imageShape'])
        image, pixel_sha = scene._image(image_bytes,r['imageShape'])
        observed = motion.observe_pair(first,image,p['movingPolygon'],p['backgroundPolygons'],1)
        scene._require(observed['valid'],'rigid_scene_tracking_rejected:'+str(observed['reason']))
        bp = base['parameters'];x0,y0,x1,y1 = bp['patchRois']['object'];a,b,c,d = bp['searchRois']['object']
        scores = cv2.matchTemplate(image[b:d,a:c],first[y0:y1,x0:x1],cv2.TM_CCOEFF_NORMED)
        scene._require(np.isfinite(scores).all(),'scene_nonfinite_match')
        obj, quality = scene._localize(scores,np.zeros(scores.shape,dtype=np.uint8),[0],base,'object')
        transform = np.asarray(observed['moving']['fit']['transform'])
        points = dict(object=obj)
        for name in ('jawLeft','jawRight'):
            points[name] = (transform[:,:2]@np.asarray(r['points'][name])+transform[:,2]).tolist()
        scene._require(all(np.isfinite(xy).all() and 0 <= xy[0] < image.shape[1]
                           and 0 <= xy[1] < image.shape[0] for xy in points.values()),
                       'rigid_scene_target_outside_frame')
        left,right = np.asarray(points['jawLeft']),np.asarray(points['jawRight'])
        opening = float(np.linalg.norm(right-left))
        annotation = dict(frameId=frame_id,pixelSha256=pixel_sha,points=points,accepted=True,inputFrameOnly=True,
            method=METHOD,registrationSha256=r['registrationSha256'],parametersSha256=r['parametersSha256'],
            referenceImageSha256=r['referenceImageSha256'],landmarkContract=self.landmark_contract)
        return dict(annotation=annotation,state=[*(np.asarray(obj)-(left+right)/2).tolist(),opening],
            quality=dict(object=quality,rigid=observed),imageSha256=scene._sha(image_bytes),pixelSha256=pixel_sha,
            sceneContext=copy.deepcopy(r['sceneContext']),trainingEligible=False,controlEligible=False,
            measurementAccuracyVerified=False,liveMotionTrackingVerified=False,depthVerified=False,contactVerified=False)
