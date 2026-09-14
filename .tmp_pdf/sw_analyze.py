# -*- coding: utf-8 -*-
import json, io, sys
from collections import Counter
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
data = json.load(open(r'C:\Users\希望\br_perception\.tmp_pdf\map_features.json', encoding='utf-8'))
for e in data:
    if e['type'] != 'ProfileFeature' or not e['subs']:
        continue
    tc = Counter(s['type'] for s in e['subs'])
    boxes = [s['box'] for s in e['subs'] if s['box']]
    print(f"{e['feature']}: subs={len(e['subs'])} types={dict(tc)}")
