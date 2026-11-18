"""Read embedded JS strings without evaluating or contacting the controller."""
import json
import re


def embedded_scripts(source):
    scripts = []
    for match in re.finditer(r'server_\.sendContent\(R"MANUALJS\((.*?)\)MANUALJS"\);|server_\.sendContent\(("[^\n]*")\);', source, re.S):
        if match[1] is not None:
            scripts.append(match[1].strip())
        else:
            try:
                scripts.append(json.loads(match[2]))
            except ValueError:
                pass
    return scripts
