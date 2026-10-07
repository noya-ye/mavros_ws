import json
import re
from pathlib import Path

ROOT = Path('/home/jetson/PX4-Autopilot')
LOG = Path('/home/jetson/.ros/log/mavros_node_10468_1791188045560.log')
ids = {int(x) for x in re.findall(r'EVENT (\d+)', LOG.read_text())}
matches = []
for path in (ROOT/'src').rglob('*'):
    if path.suffix not in ['.cpp','.h','.hpp']:
        continue
    content = path.read_text(errors='replace')
    for match in re.finditer(r'events::ID\("([^"]+)"\)',content):
        name = match[1]
        h = 0x811c9dc5
        for c in name:
            h = ((h ^ ord(c))*0x1000193)&0xffffffff
        event_id = h&0xffffff
        if event_id in ids:
            matches.append(dict(id=event_id,name=name,path=str(path),
                line=content[:match.start()].count('\n')+1,
                context=content[max(0,match.start()-150):match.end()+550]))
out = Path(__file__).resolve().parent
(out/'decoded_events.json').write_text(json.dumps(matches,indent=2))
for m in matches:
    print(json.dumps(m))
