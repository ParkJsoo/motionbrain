#!/usr/bin/env python3
"""명시한 기존 원본만 새 배경 기준 좌표계로 개발 재생한다. 촬영·장치 접근 없음."""
import argparse
import json
import hashlib
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import cv2
import numpy as np
from learning.batch_background_observation import make_spec,observe_frame
from learning.batch_observation import require,digest
from learning.batch_runtime import private_path
from learning.lossless_frame import decode
from learning.experiment_pipeline import encoded


def hashed(repository,binding):
    require(isinstance(binding,dict) and set(binding)=={'path','sha256'},'replay_file_binding')
    path=private_path(repository,binding['path']);raw=path.read_bytes()
    require(hashlib.sha256(raw).hexdigest()==binding['sha256'],'replay_original_hash_changed')
    return json.loads(raw)


def source_file(folder,name):
    require(isinstance(name,str) and not Path(name).is_absolute(),'replay_relative_source_required')
    path=(folder/name).resolve();require(path.is_relative_to(folder),'replay_source_escape')
    return path


def run(manifest_path,output,repository=ROOT):
    repository=Path(repository).resolve();manifest_path=private_path(repository,str(manifest_path));output=private_path(repository,str(output))
    require(not output.exists(),'replay_new_output_required')
    manifest=json.loads(manifest_path.read_bytes())
    require(set(manifest)=={'legacySpec','sets'} and isinstance(manifest['sets'],list) and manifest['sets'],'replay_manifest')
    spec=make_spec(hashed(repository,manifest['legacySpec']))
    output.mkdir(parents=True);(output/'observer.json').write_bytes(encoded(spec))
    summary=dict(kind='background_normalization_development_replay',specSha256=digest(spec),
        manifestSha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),inputManifest=manifest,sets=[],
        motorCommands=0,deviceAccess=False,controlEligible=False,physicalTrackingVerified=False,holdout=False)
    for entry in manifest['sets']:
        require(set(entry)=={'name','role','folder','frames','expectedViewConfigurationId','expectedConfigurationSha256'} and entry['role'] in ('development','diagnostic_development'),
                'replay_development_role_required')
        folder=private_path(repository,entry['folder']);frames=hashed(repository,entry['frames'])['frames'];rows=[]
        for frame in frames:
            # Native receipt and metadata hashes remain authoritative. Replaying
            # at its recorded persisted time is not a new live observation.
            meta_raw=source_file(folder,frame['metadataFile']).read_bytes()
            require(hashlib.sha256(meta_raw).hexdigest()==frame['metadataSha256'],'replay_metadata_changed')
            meta=json.loads(meta_raw)
            require(meta['viewConfigurationId']==entry['expectedViewConfigurationId'] and
                meta['configurationSha256']==entry['expectedConfigurationSha256'] and
                meta['deviceUniqueId']==spec['cameraIdentity'] and [meta['height'],meta['width']]==spec['imageShape'],
                'replay_acquisition_configuration_changed')
            raw=source_file(folder,frame['imageFile']).read_bytes()
            require(hashlib.sha256(raw).hexdigest()==frame['imageSha256']==meta['imageSha256'],'replay_pixels_changed')
            pixels=decode(raw,meta['rawByteLength'],meta['pixelSha256'])
            bgra=np.frombuffer(pixels,np.uint8).reshape(meta['height'],meta['width'],4)
            bgr=cv2.cvtColor(bgra,cv2.COLOR_BGRA2BGR);ok,png=cv2.imencode('.png',bgr)
            require(ok,'replay_png_encoding');image=png.tobytes()
            persisted=frame.get('storageCompleteHostSeconds',frame.get('stdoutReadAt',meta['processingCompleteHostSeconds'])+frame['persistenceMs']/1000)
            metadata=dict(frameId=meta['sourceFrameId'],imageSha256=hashlib.sha256(image).hexdigest(),
                pixelSha256=hashlib.sha256(bgr.tobytes()).hexdigest(),capturedAt=meta['convertedHostPtsSeconds'],
                persistedAt=persisted,clockId=meta['sessionId'],cameraIdentity=meta['deviceUniqueId'])
            result=observe_frame(image,metadata,spec,{k:spec[k] for k in ('viewId','poseRegime','object','load')},persisted,meta['sessionId'])
            rows.append(dict(originalImageSha256=meta['imageSha256'],originalPixelSha256=meta['pixelSha256'],
                originalMetadataSha256=frame['metadataSha256'],originalViewConfigurationId=meta['viewConfigurationId'],
                observation=result))
        name=entry['name'];require(isinstance(name,str) and name.replace('_','').isalnum(),'replay_set_name')
        (output/(name+'.json')).write_bytes(encoded(dict(role=entry['role'],rows=rows)))
        reasons={}
        for row in rows:
            reason=row['observation'].get('reason') or row['observation']['status'];reasons[reason]=reasons.get(reason,0)+1
        summary['sets'].append(dict(name=name,role=entry['role'],frames=len(rows),
            accepted=sum(r['observation']['status']=='accepted' for r in rows),reasons=reasons))
    (output/'report.json').write_bytes(encoded(summary));return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--manifest',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();print(json.dumps(run(args.manifest,args.output),ensure_ascii=False))

if __name__=='__main__':main()
