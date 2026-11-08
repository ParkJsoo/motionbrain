"""검증 상태와 근거를 보존하는 planar 팔 기하 프로필. ROS 의존성은 없다."""

import hashlib
from importlib import resources
import json
import math
from pathlib import Path


SCHEMA = "motionbrain.arm_geometry.planar.v1"
GEOMETRY_FIELDS = (
    "base_height_m", "shoulder_offset_m", "upper_arm_m",
    "forearm_m", "wrist_m", "tool_m",
)
_CONVENTION = {
    "yaw_axis": [0, 0, 1],
    "pitch_axis": [0, -1, 0],
    "joint_origin_rpy": [0, 0, 0],
    "positive_pitch": "upward_elevation",
    "tool_frame": "tool0",
    "tool_reference": "fixed_to_gripper_body",
    "jaw_motion_affects_tool": False,
}
_FIELD_KEYS = {
    "value_m", "status", "source", "method", "uncertainty_m",
    "evidence_sha256", "physical_revision",
}


def _keys(value, expected, location):
    if type(value) is not dict or set(value) != set(expected):
        raise ValueError(f"invalid_geometry_fields:{location}")


def _number(value, location, *, minimum=0.0, maximum=10.0, positive=False):
    if (type(value) not in (int, float) or value < minimum or value > maximum
            or not math.isfinite(value) or (positive and value == 0)):
        raise ValueError(f"invalid_geometry_number:{location}")


def _text(value, location):
    if not isinstance(value, str) or not value.strip() or value != value.strip():
        raise ValueError(f"invalid_geometry_text:{location}")


def _validate(profile):
    _keys(profile, {"schema", "units", "model", "convention", "status", "parameters"}, "profile")
    if profile["schema"] != SCHEMA:
        raise ValueError("unsupported_geometry_schema")
    if profile["units"] != "metres" or profile["model"] != "planar_yaw_3pitch":
        raise ValueError("unsupported_geometry_model_or_units")
    convention = profile["convention"]
    _keys(convention, _CONVENTION, "convention")
    for name, expected in _CONVENTION.items():
        actual = convention[name]
        if isinstance(expected, list):
            if type(actual) is not list or len(actual) != 3:
                raise ValueError(f"unsupported_geometry_convention:{name}")
            for value in actual:
                _number(value, f"convention.{name}", minimum=-1.0, maximum=1.0)
        elif type(actual) is not type(expected):
            raise ValueError(f"unsupported_geometry_convention:{name}")
        if actual != expected:
            raise ValueError(f"unsupported_geometry_convention:{name}")
    parameters = profile["parameters"]
    _keys(parameters, GEOMETRY_FIELDS, "parameters")
    for name in GEOMETRY_FIELDS:
        field = parameters[name]
        _keys(field, _FIELD_KEYS, name)
        # 10m is only a broad input sanity bound, never a hardware safety limit.
        _number(field["value_m"], name, positive=name in {"upper_arm_m", "forearm_m"})
        if field["status"] not in ("nominal", "measured"):
            raise ValueError(f"unsupported_geometry_status:{name}")
        _text(field["source"], f"{name}.source")
        _text(field["method"], f"{name}.method")
        uncertainty = field["uncertainty_m"]
        if uncertainty is not None:
            _number(uncertainty, f"{name}.uncertainty_m")
        evidence = field["evidence_sha256"]
        if evidence is not None and (
            not isinstance(evidence, str) or len(evidence) != 64
            or any(c not in "0123456789abcdef" for c in evidence)
        ):
            raise ValueError(f"invalid_geometry_evidence_sha256:{name}")
        revision = field["physical_revision"]
        if revision is not None:
            _text(revision, f"{name}.physical_revision")
        if field["status"] == "measured":
            # A coarse manual reading is still a measurement. Keep unknown
            # uncertainty explicit rather than inventing a numeric precision.
            if evidence is None or revision is None:
                raise ValueError(f"missing_measured_geometry_evidence:{name}")
            if (field["method"] in {"not_measured", "unknown"}
                    or field["source"] in {"legacy_unverified_default", "unknown"}):
                raise ValueError(f"missing_measured_geometry_method:{name}")
    least_status = "measured" if all(
        parameters[name]["status"] == "measured" for name in GEOMETRY_FIELDS
    ) else "nominal"
    if profile["status"] != least_status:
        raise ValueError("geometry_profile_status_mismatch")
    measured_revisions = {
        parameters[name]["physical_revision"] for name in GEOMETRY_FIELDS
        if parameters[name]["status"] == "measured"
    }
    if len(measured_revisions) > 1:
        raise ValueError("geometry_physical_revision_mismatch")


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate_geometry_key:{key}")
        result[key] = value
    return result


def _invalid_constant(value):
    raise ValueError(f"nonfinite_geometry_json:{value}")


def load_geometry_profile(path=None) -> dict:
    """기본 설치 resource 또는 명시한 JSON을 읽는다. 실패 시 대체하지 않는다."""
    raw = (resources.files(__package__).joinpath("arm_geometry.json").read_text(encoding="utf-8")
           if path is None else Path(path).read_text(encoding="utf-8"))
    profile = json.loads(raw, object_pairs_hook=_unique_object, parse_constant=_invalid_constant)
    _validate(profile)
    return profile


def geometry_values(profile) -> dict:
    """검증된 프로필에서 ArmModel의 여섯 길이만 반환한다."""
    _validate(profile)
    return {name: float(profile["parameters"][name]["value_m"]) for name in GEOMETRY_FIELDS}


def profile_digest(profile) -> str:
    """값과 근거·상태를 함께 묶는 정렬된 JSON SHA-256."""
    _validate(profile)
    encoded = json.dumps(profile, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=False, allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
