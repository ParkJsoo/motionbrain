"""Render the supported planar skeleton without changing control contracts.

URDFs remain checked-in derivatives. This module owns the supported skeleton,
approximate display visuals and mirrored display jaw. ros2_control and physical
contracts are preserved; camera extrinsics and all dynamics remain uncalibrated.
"""
from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET

from motionbrain_ros_bridge.arm_geometry import geometry_values
from motionbrain_ros_bridge.arm_appearance import build_visuals


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
    prefix = r'<' + tag + r'\s+name="' + re.escape(name) + r'"'
    pattern = re.compile(prefix + suffix + r'(?<!/)>.*?</' + tag + r'>', re.S)
    matches = list(pattern.finditer(text))
    empty = list(re.finditer(prefix + r'[^>]*?/\s*>', text)) if tag == 'link' else []
    if len(matches) + len(empty) != 1:
        raise ValueError('expected_one_' + tag + ':' + name)
    match = (matches or empty)[0]
    block = match.group()
    if empty:
        block = re.sub(r'/\s*>$', '></' + tag + '>', block)
    return text[:match.start()] + transform(block) + text[match.end():]


def _origin(block, xyz):
    current = ET.fromstring(block).find('origin')
    if current is not None and _vector(current.get('xyz', '0 0 0')) == xyz:
        return block
    element = '<origin xyz="' + _text(xyz) + '" rpy="0 0 0"/>'
    if re.search(r'<origin\b[^>]*/>', block):
        return re.sub(r'<origin\b[^>]*/>', element, block, count=1)
    return block.replace('</joint>', '  ' + element + '\n  </joint>', 1)


def _appearance(text, values):
    """Own display visuals only; leave inertia/collision/control text intact."""
    root = ET.fromstring(text)
    if root.find("link[@name='gripper_opposing_jaw_link']") is None:
        text = text.replace('</robot>', '<link name="gripper_opposing_jaw_link"></link>\n</robot>')
    text = _replace_block(text, 'joint', 'gripper_joint', lambda b: _origin(b, [.01, .012, 0]))
    limit = root.find("joint[@name='gripper_joint']/limit")
    follower = ET.Element('joint', name='gripper_opposing_joint', type='revolute')
    ET.SubElement(follower, 'parent', link='gripper_link')
    ET.SubElement(follower, 'child', link='gripper_opposing_jaw_link')
    ET.SubElement(follower, 'origin', xyz='.01 -.012 0', rpy='0 0 0')
    ET.SubElement(follower, 'axis', xyz='0 0 1')
    follower_limit = dict(limit.attrib, lower=format(-float(limit.get('upper')), '.12g'),
                          upper=format(-float(limit.get('lower')), '.12g'))
    ET.SubElement(follower, 'limit', **follower_limit)
    ET.SubElement(follower, 'mimic', joint='gripper_joint', multiplier='-1', offset='0')
    ET.indent(follower)
    xml = ET.tostring(follower, encoding='unicode')
    if root.find("joint[@name='gripper_opposing_joint']") is None:
        text = text.replace('</robot>', xml + '\n</robot>')
    else:
        text = _replace_block(text, 'joint', 'gripper_opposing_joint', lambda _: xml)
    visual_map = build_visuals(values)
    palette = {}
    for elements in visual_map.values():
        for visual in elements:
            material = visual.find('material')
            color = material.find('color').get('rgba')
            if color not in palette:
                palette[color] = 'owi_palette_' + str(len(palette))
            material.set('name', palette[color])
    # urdfdom/RViz resolves all visual-array materials reliably when the names
    # are declared globally, rather than only inline on later visuals.
    text = re.sub(r'\s*<material\s+name="owi_palette_\d+"[^>]*>.*?</material>', '', text, flags=re.S)
    declarations = []
    for color, name in palette.items():
        material = ET.Element('material', name=name)
        ET.SubElement(material, 'color', rgba=color)
        ET.indent(material, level=1)
        declarations.append('  ' + ET.tostring(material, encoding='unicode'))
    text = re.sub(r'(<robot\b[^>]*>)', lambda m: m[1] + '\n' + '\n'.join(declarations), text, count=1)
    for name, elements in visual_map.items():
        def replace(block, visuals=elements):
            block = re.sub(r'\s*<visual\b[^>]*>.*?</visual>', '', block, flags=re.S)
            block = re.sub(r'\s*</link>', '</link>', block)
            rendered = []
            for visual in visuals:
                ET.indent(visual, level=2)
                rendered.append('    ' + ET.tostring(visual, encoding='unicode').rstrip())
            return block.replace('</link>', '\n' + '\n'.join(rendered) + '\n  </link>')
        text = _replace_block(text, 'link', name, replace)
    return text


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
    # Control-only descriptions retain their original joint/control contracts.
    if full and (root.get('name')=='motionbrain' or root.find('.//visual') is not None):
        result = _appearance(result, values)
    return result


def geometry_mismatches(text, profile):
    """Numerical comparison ignores formatting but detects origins and visuals."""
    actual = ET.fromstring(text)
    expected = ET.fromstring(render_urdf(text, profile))
    errors = []
    def palette(root):
        return {m.get('name'): m.find('color').get('rgba') for m in root.findall('material')
                if m.get('name','').startswith('owi_palette_') and m.find('color') is not None}
    if palette(actual) != palette(expected):
        errors.append('appearance_palette')
    for node in expected.findall('joint'):
        name = node.get('name')
        other = actual.find(f"joint[@name='{name}']")
        if other is None:
            errors.append(name + '/missing_joint')
            continue
        for tag, attr in [('origin', 'xyz'), ('origin', 'rpy')]:
            a, b = other.find(tag), node.find(tag)
            va = _vector(a.get(attr, '0 0 0')) if a is not None else [0, 0, 0]
            vb = _vector(b.get(attr, '0 0 0')) if b is not None else [0, 0, 0]
            if not all(math.isclose(x, y, abs_tol=1e-11, rel_tol=0) for x, y in zip(va, vb)):
                errors.append(name + '/' + tag + '/' + attr)
        for tag in ('parent', 'child', 'axis', 'limit'):
            a, b = other.find(tag), node.find(tag)
            if ((a is None) != (b is None) or
                    (a is not None and b is not None and a.attrib != b.attrib)):
                errors.append(name + '/' + tag)
        actual_mimic, expected_mimic = other.find('mimic'), node.find('mimic')
        if ((actual_mimic is None) != (expected_mimic is None) or
                (actual_mimic is not None and expected_mimic is not None and
                 actual_mimic.attrib != expected_mimic.attrib)):
            errors.append(name + '/mimic')
    for node in expected.findall('link'):
        name = node.get('name')
        other = actual.find(f"link[@name='{name}']")
        if other is None:
            errors.append(name + '/missing_link')
            continue
        def signature(link):
            def node_signature(n):
                attrs = []
                for key, value in sorted(n.attrib.items()):
                    try:
                        value = tuple(round(float(v), 11) for v in value.split())
                    except ValueError:
                        pass
                    attrs.append((key, value))
                return n.tag, tuple(attrs), tuple(node_signature(child) for child in n)
            return tuple(node_signature(v) for v in link.findall('visual'))
        if signature(other) != signature(node):
            errors.append(name + '/visuals')
    return errors
