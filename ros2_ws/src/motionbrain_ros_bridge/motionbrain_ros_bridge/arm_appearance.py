"""OWI-535 사진·조립도를 참고한 절차적 외형 근사.

관절 좌표와 FK를 바꾸지 않는 visual만 만든다. 레일 길이는 입력 skeleton을
따르지만 케이스·나사·카메라 지지대의 세부 치수는 실측값이 아니다.
충돌 형상, 질량, 관성, 카메라 보정, 실제 집게 접촉 모델을 제공하지 않는다.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET


_YELLOW = (1.0, .72, .015, 1)
_BLACK = (.065, .07, .075, 1)
_DARK = (.025, .028, .032, 1)
_SILVER = (.72, .75, .77, 1)
_WHITE = (.93, .93, .89, 1)
_PAD = (.30, .32, .32, 1)
_FIELDS = ("base_height_m", "shoulder_offset_m", "upper_arm_m", "forearm_m", "wrist_m", "tool_m")


def _numbers(values):
    return " ".join(format(float(value), ".12g") for value in values)


def _visual(name, shape, dimensions, xyz, color, rpy=(0, 0, 0)):
    visual = ET.Element("visual", name="owi_" + name)
    ET.SubElement(visual, "origin", xyz=_numbers(xyz), rpy=_numbers(rpy))
    geometry = ET.SubElement(visual, "geometry")
    if shape == "box":
        ET.SubElement(geometry, "box", size=_numbers(dimensions))
    elif shape == "cylinder":
        ET.SubElement(geometry, "cylinder", radius=_numbers(dimensions[:1]),
                      length=_numbers(dimensions[1:]))
    elif shape == "sphere":
        ET.SubElement(geometry, "sphere", radius=_numbers(dimensions))
    else:
        raise ValueError("unsupported_appearance_primitive")
    material = ET.SubElement(visual, "material", name="owi_material_" + name)
    ET.SubElement(material, "color", rgba=_numbers(color))
    return visual


def _box(name, size, xyz, color, rpy=(0, 0, 0)):
    return _visual(name, "box", size, xyz, color, rpy)


def _cylinder(name, radius, length, xyz, color, rpy=(0, 0, 0)):
    return _visual(name, "cylinder", (radius, length), xyz, color, rpy)


def _gearcase(prefix, x, radius=.025, width=.042):
    """옆면이 보이는 노랑 플랜지·검정 캡. 축은 local Y 방향이다."""
    result = [
        _box(prefix + "_housing", (.037, width, .035), (x, 0, 0), _YELLOW),
    ]
    for side, label in [(-1, "near"), (1, "far")]:
        y = side * (width / 2)
        result.extend([
            _cylinder(prefix + "_" + label + "_flange", radius, .004,
                      (x, y, 0), _YELLOW, (math.pi / 2, 0, 0)),
            _cylinder(prefix + "_" + label + "_cap", radius * .78, .006,
                      (x, y + side * .003, 0), _BLACK, (math.pi / 2, 0, 0)),
            _cylinder(prefix + "_" + label + "_hub", .006, .008,
                      (x, y + side * .006, 0), _DARK, (math.pi / 2, 0, 0)),
            _cylinder(prefix + "_" + label + "_pin", .0025, .009,
                      (x, y + side * .007, 0), _SILVER, (math.pi / 2, 0, 0)),
        ])
        for index in range(3):
            angle = index * 2 * math.pi / 3
            result.append(_cylinder(
                prefix + "_" + label + f"_screw_{index}", .0017, .004,
                (x + radius * .69 * math.cos(angle), y + side * .007,
                 radius * .69 * math.sin(angle)), _SILVER, (math.pi / 2, 0, 0)))
    return result


def _rails(prefix, length, half_width=.022, height=.013):
    result = []
    if length <= 0:
        return result
    for side, label in [(-1, "near"), (1, "far")]:
        result.append(_box(prefix + "_rail_" + label,
                           (length, .006, height), (length / 2, side * half_width, 0), _BLACK))
        result.append(_box(prefix + "_rib_" + label,
                           (length * .86, .002, .003), (length / 2, side * (half_width + .004), 0), _DARK))
    for index, fraction in enumerate((.12, .88)):
        result.append(_box(prefix + f"_crossbar_{index}", (.007, half_width * 2, height * .7),
                           (length * fraction, 0, 0), _BLACK))
    return result


def _jaw(prefix, length, side):
    # Both fingers extend along their joint-local +X. Mirroring is a joint
    # contract owned by the description/exporter, not hidden in these visuals.
    if length <= 0:
        return []
    result = [
        _box(prefix + "_finger", (length, .007, .010), (length / 2, 0, 0), _BLACK),
        _box(prefix + "_hinge_plate", (min(.017, length), .013, .006),
             (min(.007, length / 2), 0, .003), _DARK),
        _box(prefix + "_inner_pad", (length * .54, .002, .011),
             (length * .72, -side * .0045, 0), _PAD),
        _cylinder(prefix + "_pin", .0024, .010, (0, 0, .005), _SILVER),
    ]
    for index in range(6):
        result.append(_box(prefix + f"_rib_{index}", (.0015, .008, .012),
                           (length * (.25 + index * .12), 0, 0), _DARK))
    return result


def build_visuals(values) -> dict[str, list[ET.Element]]:
    """동일한 scalar 입력에 동일한 독립 ET visual 목록을 반환한다.

    gripper_link 원점은 wrist_m 위치의 명목상 고정 몸체 기준이다.
    jaw joint 원점 (0.01, +/-0.012, 0)과 합쳐 각 손가락 끝이 tool_m에
    오도록 만든다. legacy camera_link는 외형을 부여하지 않는다.
    """
    for name in _FIELDS:
        value = values[name]
        if type(value) not in (int, float) or not 0 <= value <= 10 or not math.isfinite(value):
            raise ValueError("invalid_appearance_geometry:" + name)
    if values["upper_arm_m"] == 0 or values["forearm_m"] == 0:
        raise ValueError("invalid_appearance_link_length")
    height, offset, upper, forearm, wrist, tool = (float(values[name]) for name in _FIELDS)
    result = {name: [] for name in (
        "base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_link",
        "gripper_link", "gripper_jaw_link", "gripper_opposing_jaw_link", "camera_link")}

    # Fixed base with the unmistakable yellow horizontal stack and black rear tray.
    stack = min(.043, height * .62)
    base = result["base_link"]
    base.extend([
        _cylinder("base_bottom", .067, .006, (0, 0, .003), _DARK),
        _cylinder("base_core", .058, max(stack, .003), (0, 0, .006 + stack / 2), _BLACK),
        _box("base_rear_tray", (.105, .080, .042), (-.072, 0, .024), _BLACK),
        _box("base_rear_lid", (.106, .081, .005), (-.072, 0, .0475), _DARK),
    ])
    for index in range(3):
        z = .010 + stack * index / 3
        base.append(_cylinder(f"base_yellow_ring_{index}", .065, .0045, (0, 0, z), _YELLOW))
    for side, label in [(-1, "near"), (1, "far")]:
        base.append(_box("base_foot_" + label, (.14, .012, .005),
                         (-.006, side * .069, .0025), _BLACK))
        for index in range(3):
            base.append(_box(f"base_stack_post_{label}_{index}", (.006, .006, max(stack, .003)),
                             (-.041 + index * .041, side * .048, .006 + stack / 2), _YELLOW))

    shoulder = result["shoulder_link"]
    shoulder.extend(_gearcase("m4", offset, .026, .044))
    if offset > 0:
        shoulder.extend(_rails("shoulder_connector", offset, .017, .012))
    shoulder.extend([
        _box("m4_sensor_plate", (.029, .003, .029), (offset, -.034, 0), _SILVER),
        _box("m4_sensor_chip", (.007, .005, .007), (offset, -.037, 0), _DARK),
    ])

    result["upper_arm_link"].extend(_rails("upper", upper, .024, .015))
    for x, label in [(0, "proximal"), (upper, "distal")]:
        result["upper_arm_link"].append(_cylinder("upper_" + label + "_axle", .004, .061,
                                                        (x, 0, 0), _SILVER, (math.pi / 2, 0, 0)))

    # M3 and M2 motor cases belong to the forearm rigid assembly. The M2 case
    # sits behind the true external wrist hinge; it is not a replacement joint.
    fore = result["forearm_link"]
    fore.extend(_rails("forearm", forearm, .020, .010))
    fore.extend(_gearcase("m3", 0, .024, .039))
    m2_case_x = max(0, forearm - .040)
    fore.extend(_gearcase("m2_motor", m2_case_x, .023, .038))
    if forearm > m2_case_x:
        extension = forearm - m2_case_x
        for side, label in [(-1, "near"), (1, "far")]:
            fore.append(_box("m2_fixed_bracket_" + label, (extension, .005, .022),
                             (m2_case_x + extension / 2, side * .022, 0), _BLACK))

    wrist_visuals = result["wrist_link"]
    wrist_visuals.extend([
        _cylinder("wrist_external_hinge", .014, .048, (0, 0, 0), _BLACK, (math.pi / 2, 0, 0)),
        _cylinder("wrist_external_pin", .0032, .052, (0, 0, 0), _SILVER, (math.pi / 2, 0, 0)),
    ])
    wrist_visuals.extend(_rails("wrist_body_support", wrist, .016, .012))

    grip = result["gripper_link"]
    grip.extend([
        _box("gripper_fixed_body", (.048, .036, .025), (-.011, 0, 0), _BLACK),
        _box("gripper_top_cover", (.041, .034, .004), (-.012, 0, .0145), _DARK),
        _box("gripper_rear_motor", (.022, .031, .020), (-.038, 0, 0), _BLACK),
    ])
    for side, label in [(-1, "near"), (1, "far")]:
        grip.extend([
            _cylinder("gripper_gear_" + label, .009, .006, (.003, side * .012, .014), _BLACK),
            _cylinder("gripper_gear_pin_" + label, .002, .007, (.003, side * .012, .017), _SILVER),
        ])
    # A visual-only approximation of the user's white raised camera bracket.
    grip.extend([
        _box("camera_white_foot", (.044, .013, .004), (-.012, 0, .020), _WHITE),
        _box("camera_white_upright", (.005, .013, .082), (-.029, 0, .062), _WHITE),
        _box("camera_white_arm", (.044, .013, .005), (-.009, 0, .104), _WHITE),
        _box("camera_board", (.010, .032, .024), (.011, 0, .111), _DARK),
        _box("camera_white_wrap", (.006, .028, .023), (.006, 0, .111), _WHITE),
        _cylinder("camera_lens", .006, .009, (.020, 0, .111), _BLACK, (0, math.pi / 2, 0)),
    ])
    result["gripper_jaw_link"] = _jaw("jaw_left", max(0, tool - .01), 1)
    result["gripper_opposing_jaw_link"] = _jaw("jaw_right", max(0, tool - .01), -1)
    return result
