"""일부 RViz 버전의 링크 첫 재질 공유 문제를 피하는 표시 전용 변환."""
import copy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def rviz_description(text):
    """색별 고정 표시 링크로 나누며 기존 관절·도구·visual 위치는 보존한다.

    물리 모델 생성에는 이 파생물을 사용하지 않는다. 추가 링크는 표시용
    TF이며 실제 부품·관절·센서 프레임이 아니다.
    """
    root = ET.fromstring(text)
    if any(l.get('name','').startswith('rviz_visual_') for l in root.findall('link')):
        raise ValueError('rviz_appearance_already_expanded')
    for link in list(root.findall('link')):
        groups = {}
        for visual in list(link.findall('visual')):
            material = visual.find('material')
            color = material.find('color') if material is not None else None
            key = color.get('rgba') if color is not None else (material.get('name') if material is not None else '')
            groups.setdefault(key, []).append(copy.deepcopy(visual))
            link.remove(visual)
        for i, elements in enumerate(groups.values()):
            name = f"rviz_visual_{link.get('name')}_{i}"
            child = ET.SubElement(root, 'link', name=name)
            child.extend(elements)
            joint = ET.SubElement(root, 'joint', name=name+'_mount', type='fixed')
            ET.SubElement(joint, 'parent', link=link.get('name'))
            ET.SubElement(joint, 'child', link=name)
            ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0')
    ET.indent(root)
    return ET.tostring(root, encoding='unicode')


if __name__ == '__main__':
    print(rviz_description(Path(sys.argv[1]).read_text()))
