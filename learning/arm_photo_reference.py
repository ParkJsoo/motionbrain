"""측면 사진의 표식으로 만드는 오프라인 근사 자세. 실물 보정이 아니다."""
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np

from motionbrain_ros_bridge.arm_geometry import (
    geometry_values, load_geometry_profile, profile_digest,
)


_NAMES = ("shoulder", "elbow", "wrist", "tool_tip")
_FIELDS = {"imagePath", "imageSha256", "imageWidth", "imageHeight", "points",
           "forwardImageSign", "horizontalRollDeg", "geometryProfilePath", "geometryProfileSha256"}


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _finite(value):
    return type(value) in (int, float) and -1e9 <= value <= 1e9 and math.isfinite(value)


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _path(root, value):
    _require(isinstance(value, str) and bool(value.strip()), "invalid_reference_path")
    path = Path(value)
    return (path if path.is_absolute() else root / path).resolve(strict=True)


def _image_size(raw):
    # Read only image headers: no camera, capture process, or image re-encoding.
    if raw.startswith(b"\x89PNG\r\n\x1a\n") and len(raw) >= 24 and raw[12:16] == b"IHDR":
        return struct.unpack(">II", raw[16:24])
    if raw.startswith(b"\xff\xd8"):
        index = 2
        while index < len(raw):
            _require(raw[index] == 255, "invalid_reference_jpeg")
            while index < len(raw) and raw[index] == 255:
                index += 1
            _require(index < len(raw), "invalid_reference_jpeg")
            marker = raw[index]
            index += 1
            if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
                continue
            _require(index + 2 <= len(raw), "invalid_reference_jpeg")
            size = int.from_bytes(raw[index:index + 2], "big")
            _require(size >= 2 and index + size <= len(raw), "invalid_reference_jpeg")
            if marker in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
                          0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
                _require(size >= 8, "invalid_reference_jpeg")
                height, width = struct.unpack(">HH", raw[index + 3:index + 7])
                return width, height
            _require(marker not in (0xDA, 0xD9), "missing_reference_jpeg_size")
            index += size
    raise ValueError("unsupported_reference_image_header")


def infer_reference(manifest, root) -> dict:
    """모든 근거 해시를 검사하고 측면·동일 평면 약투영 가정의 후보를 반환한다.

    horizontalRollDeg는 영상의 오른쪽 방향 수평선이 image-up 좌표계에서
    반시계로 기운 각도이다. 이를 제거한 뒤 forwardImageSign을 적용한다.
    geometryProfileSha256는 파일 바이트가 아닌 profile_digest의 canonical SHA다.
    """
    _require(type(manifest) is dict and set(manifest) == _FIELDS, "invalid_reference_fields")
    root = Path(root).resolve(strict=True)
    image = _path(root, manifest["imagePath"])
    raw = image.read_bytes()
    _require(_sha(raw) == manifest["imageSha256"], "reference_image_hash_mismatch")
    width, height = manifest["imageWidth"], manifest["imageHeight"]
    _require(type(width) is int and type(height) is int and width > 0 and height > 0,
             "invalid_reference_image_dimensions")
    _require(_image_size(raw) == (width, height), "reference_image_dimensions_mismatch")
    points = manifest["points"]
    _require(type(points) is dict and set(points) == set(_NAMES), "invalid_reference_points")
    for name in _NAMES:
        point = points[name]
        _require(type(point) is list and len(point) == 2 and all(_finite(v) for v in point),
                 "invalid_reference_point:" + name)
        _require(0 <= point[0] < width and 0 <= point[1] < height,
                 "reference_point_outside_image:" + name)
    sign, roll = manifest["forwardImageSign"], manifest["horizontalRollDeg"]
    _require(type(sign) is int and sign in (-1, 1), "invalid_reference_forward_sign")
    _require(_finite(roll) and abs(roll) <= 180, "invalid_reference_roll")
    geometry_path = _path(root, manifest["geometryProfilePath"])
    geometry_raw = geometry_path.read_bytes()
    profile = load_geometry_profile(geometry_path)
    _require(geometry_path.read_bytes() == geometry_raw, "reference_geometry_changed_during_read")
    _require(profile_digest(profile) == manifest["geometryProfileSha256"], "reference_geometry_hash_mismatch")
    values = geometry_values(profile)
    lengths = np.array([values["upper_arm_m"], values["forearm_m"], values["wrist_m"] + values["tool_m"]])
    _require(bool(np.all(lengths >= 1e-6)), "degenerate_reference_geometry")
    pixels = np.array([points[name] for name in _NAMES], dtype=float)
    vectors = np.diff(pixels, axis=0)
    distances = np.linalg.norm(vectors, axis=1)
    _require(bool(np.all(distances >= 2)), "degenerate_reference_segment")
    radians = math.radians(roll)
    dx, up = vectors[:, 0], -vectors[:, 1]
    forward = sign * (math.cos(radians) * dx + math.sin(radians) * up)
    elevation = -math.sin(radians) * dx + math.cos(radians) * up
    absolute = np.arctan2(elevation, forward)
    relative = np.array([absolute[0], absolute[1] - absolute[0], absolute[2] - absolute[1]])
    scale = float(np.dot(lengths, distances) / np.dot(lengths, lengths))
    residual_m = distances / scale - lengths
    projected = np.vstack([pixels[0], pixels[0] + np.cumsum(
        vectors / distances[:, None] * (scale * lengths)[:, None], axis=0)])
    names = ("upper_arm", "forearm", "wrist_to_observed_tip")
    canonical = json.dumps(manifest, sort_keys=True, separators=(",", ":"),
                           ensure_ascii=False, allow_nan=False).encode()
    return {
        "schema": "motionbrain.arm_photo_reference.v1",
        "status": "coarse_photo_reference_only",
        "evidence": {"imageSha256": _sha(raw), "geometryProfileSha256": profile_digest(profile),
                     "geometryFileSha256": _sha(geometry_raw), "manifestCanonicalSha256": _sha(canonical)},
        "assumptions": ["same_plane_side_view", "weak_perspective_shared_scale",
                        "declared_horizontal_roll", "declared_forward_direction",
                        "direct_principal_elevation_differences_not_joint_winding",
                        "observed_surface_tip_is_not_fixed_tool0"],
        "image": {"width": width, "height": height,
                  "points": {name: list(points[name]) for name in _NAMES}},
        "absoluteElevationDeg": dict(zip(names, map(float, np.degrees(absolute)))),
        "jointCandidateDeg": dict(zip(("shoulder_pitch", "elbow_pitch", "wrist_pitch"),
                                      map(float, np.degrees(relative)))),
        "unmeasuredJoints": ["base_yaw", "gripper"],
        "scaleFit": {"pixelsPerMetre": scale,
                     "independentPixelsPerMetre": dict(zip(names, map(float, distances / lengths))),
                     "segmentLengthResidualMm": dict(zip(names, map(float, residual_m * 1000))),
                     "maxRelativeLengthResidual": float(np.max(np.abs(residual_m / lengths))),
                     "rmsLengthResidualMm": float(np.sqrt(np.mean(residual_m ** 2)) * 1000),
                     "landmarkResidualMm": dict(zip(_NAMES, map(float,
                         np.linalg.norm(projected - pixels, axis=1) / scale * 1000)))},
        "physicallyCalibrated": False, "feedbackCalibrated": False,
        "m4ZeroEstablished": False, "directionSignEstablished": False,
        "cameraIntrinsicsCalibrated": False, "cameraExtrinsicsCalibrated": False,
        "controlEligible": False,
    }
