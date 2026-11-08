"""Render the supported planar skeleton without changing control contracts.

URDFs remain checked-in derivatives. Only kinematic origins and longitudinal
schematic visuals are owned here; camera extrinsics, jaw geometry, limits,
ros2_control and all physical properties remain separate and uncalibrated.
"""
from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET

from motionbrain_ros_bridge.arm_geometry import geometry_values


URDF_PATHS = (
    'motionbrain_description/urdf/motionbrain.urdf',
    'motionbrain_ros2_control_mock/urdf/motionbrain_mock_control.urdf',
    'motionbrain_hardware_interface/urdf/motionbrain_hardware_interface.urdf',
    'motionbrain_hardware_interface/urdf/motionbrain_m4_measured.urdf',
    'motionbrain_hardware_interface/urdf/motionbrain_m4_proposal.urdf',
)


def _vector(value):
    result = [float(v) for v in value.split()]
    if len(result) != 3 or not all(math.isfinite(v) for v in result):
        raise ValueError('invalid_urdf_vector')
    return result


def _text(values):
    return ' '.join(format(v, '.12g') for v in values)


def _replace_block(text, tag, name, transform):
    # A type attribute selects the top-level kinematic joint rather than the
    # ros2_control joint of the same name. Preserve that block byte-for-byte.
    suffix = r'[^>]*\btype="[^"]+"[^>]*' if tag == 'joint' else r'[^>]*'
    pattern = re.compile(r'<' + tag + r'\s+name="' + re.escape(name) + r'"' + suffix + r'>.*?</' + tag + r'>', re.S)
    matches = list(pattern.finditer(text))
    if len(matches) != 1:
        raise ValueError('expected_one_' + tag + ':' + name)
    match = matches[0]
    return text[:match.start()] + transform(match.group()) + text[match.end():]


def _origin(block, xyz):
    current = ET.fromstring(block).find('origin')
    if current is not None and _vector(current.get('xyz', '0 0 0')) == xyz:
        return block
    element = '<origin xyz="' + _text(xyz) + '" rpy="0 0 0"/>'
    if re.search(r'<origin\b[^>]*/>', block):
        return re.sub(r'<origin\b[^>]*/>', element, block, count=1)
    return block.replace('</joint>', '  ' + element + '\n  </joint>', 1)


def _visual(block, length):
    pattern = re.compile(r'<visual\b[^>]*>.*?</visual>', re.S)
    return pattern.sub(lambda m: _visual_element(m.group(), length), block, count=1)


def _visual_element(block, length):
    # Zero offset is supported kinematically. The schematic connector is then
    # drawn as a 1mm sliver to avoid invalid zero-sized simulator geometry.
    displayed = max(length, .001)
    visual = ET.fromstring(block)
    origin = visual.find('origin')
    shape = list(visual.find('geometry'))[0]
    if origin is None:
        raise ValueError('schematic_visual_origin_missing')
    xyz = _vector(origin.get('xyz', '0 0 0'))
    original_xyz = xyz.copy()
    xyz[0] = length / 2
    if xyz != original_xyz:
        block = re.sub(r'(<origin\b[^>]*\bxyz=")[^"]*(")',
                       lambda m: m[1] + _text(xyz) + m[2], block, count=1)
    if shape.tag == 'box':
        size = _vector(shape.get('size'))
        original_size = size.copy()
        size[0] = displayed
        if size != original_size:
            block = re.sub(r'(<box\b[^>]*\bsize=")[^"]*(")',
                           lambda m: m[1] + _text(size) + m[2], block, count=1)
    elif shape.tag == 'cylinder':
        if float(shape.get('length')) != displayed:
            block = re.sub(r'(<cylinder\b[^>]*\blength=")[^"]*(")',
                           lambda m: m[1] + format(displayed, '.12g') + m[2], block, count=1)
    else:
        raise ValueError('unsupported_schematic_visual')
    return block


def render_urdf(template, profile):
    """Return a derivative; refuse unmodelled transforms instead of losing them."""
    values = geometry_values(profile)
    root = ET.fromstring(template)
    joints = {j.get('name'): j for j in root.findall('joint')}
    full = 'base_yaw_joint' in joints
    parents = {
        'base_yaw_joint': ('base_link', 'shoulder_link'),
        'shoulder_pitch_joint': ('shoulder_link', 'upper_arm_link'),
        'elbow_pitch_joint': ('upper_arm_link', 'forearm_link'),
        'wrist_pitch_joint': ('forearm_link', 'wrist_link'),
        'wrist_to_gripper': ('wrist_link', 'gripper_link'),
        'gripper_to_tool0': ('gripper_link', 'tool0'),
        'world_to_shoulder': ('world', 'shoulder_link'),
        'world_to_base': ('world', 'base_link'),
    }
    required = ('base_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint',
                'wrist_pitch_joint', 'wrist_to_gripper', 'gripper_to_tool0', 'world_to_base') if full else (
                    'world_to_shoulder', 'shoulder_pitch_joint')
    for name in required:
        if name not in joints:
            raise ValueError('missing_kinematic_joint:' + name)
        joint = joints[name]
        if tuple(joint.find(tag).get('link') for tag in ('parent', 'child')) != parents[name]:
            raise ValueError('unsupported_joint_connection:' + name)
        origin = joint.find('origin')
        if origin is not None and _vector(origin.get('rpy', '0 0 0')) != [0, 0, 0]:
            raise ValueError('unsupported_joint_rotation:' + name)
        xyz = _vector(origin.get('xyz', '0 0 0')) if origin is not None else [0, 0, 0]
        allowed = 2 if name in ('base_yaw_joint', 'world_to_shoulder') else 0
        if any(value != 0 for axis, value in enumerate(xyz) if axis != allowed) or (
                name == 'world_to_base' and any(xyz)):
            raise ValueError('unsupported_joint_translation:' + name)
        expected_type = 'revolute' if name.endswith(('_yaw_joint', '_pitch_joint')) else 'fixed'
        if joint.get('type') != expected_type:
            raise ValueError('unsupported_joint_type:' + name)
        if expected_type == 'revolute':
            expected = [0, 0, 1] if name == 'base_yaw_joint' else [0, -1, 0]
            if _vector(joint.find('axis').get('xyz')) != expected:
                raise ValueError('unsupported_joint_axis:' + name)
    origins = {
        'base_yaw_joint' if full else 'world_to_shoulder': [0, 0, values['base_height_m']],
        'shoulder_pitch_joint': [values['shoulder_offset_m'], 0, 0],
    }
    if full:
        origins.update(elbow_pitch_joint=[values['upper_arm_m'], 0, 0],
                       wrist_pitch_joint=[values['forearm_m'], 0, 0],
                       wrist_to_gripper=[values['wrist_m'], 0, 0],
                       gripper_to_tool0=[values['tool_m'], 0, 0])
    result = template
    for name, xyz in origins.items():
        result = _replace_block(result, 'joint', name, lambda b, v=xyz: _origin(b, v))
    if full:
        for link, field in [('shoulder_link', 'shoulder_offset_m'), ('upper_arm_link', 'upper_arm_m'),
                            ('forearm_link', 'forearm_m'), ('wrist_link', 'wrist_m'), ('gripper_link', 'tool_m')]:
            element = root.find(f"link[@name='{link}']")
            # Hardware/control-only descriptions intentionally have no visuals.
            if element is not None and element.find('visual') is not None:
                result = _replace_block(result, 'link', link, lambda b, v=values[field]: _visual(b, v))
    return result


def geometry_mismatches(text, profile):
    """Numerical comparison ignores formatting but detects origins and visuals."""
    actual = ET.fromstring(text)
    expected = ET.fromstring(render_urdf(text, profile))
    errors = []
    for node in expected.findall('joint'):
        name = node.get('name')
        other = actual.find(f"joint[@name='{name}']")
        for tag, attr in [('origin', 'xyz')]:
            a, b = other.find(tag), node.find(tag)
            va = _vector(a.get(attr, '0 0 0')) if a is not None else [0, 0, 0]
            vb = _vector(b.get(attr, '0 0 0')) if b is not None else [0, 0, 0]
            if not all(math.isclose(x, y, abs_tol=1e-11, rel_tol=0) for x, y in zip(va, vb)):
                errors.append(name + '/' + tag)
    for node in expected.findall('link'):
        name = node.get('name')
        other = actual.find(f"link[@name='{name}']")
        for a, b in zip(other.findall('visual'), node.findall('visual')):
            for query, attr in [('origin', 'xyz'), ('geometry/box', 'size'), ('geometry/cylinder', 'length')]:
                x, y = a.find(query), b.find(query)
                if x is None or y is None:
                    continue
                vx, vy = list(map(float, x.get(attr).split())), list(map(float, y.get(attr).split()))
                if not all(math.isclose(i, j, abs_tol=1e-11, rel_tol=0) for i, j in zip(vx, vy)):
                    errors.append(name + '/' + query)
    return errors
