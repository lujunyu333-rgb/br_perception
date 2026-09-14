# -*- coding: utf-8 -*-
import json, io, sys
from itertools import combinations
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
data = json.load(open(r'C:\Users\希望\br_perception\.tmp_pdf\sketch_segments.json', encoding='utf-8'))
TOL = 0.005

def snap(v): return round(v, 3)

def rects_from_segs(segs):
    hs, vs, others = [], [], []
    for g in segs:
        p1, p2 = g['p1'], g['p2']
        if not p1 or not p2: continue
        (x1,y1,_),(x2,y2,_) = p1, p2
        if g['t'] == 'L':
            if abs(x1-x2) < TOL: vs.append((snap(x1), min(snap(y1),snap(y2)), max(snap(y1),snap(y2))))
            elif abs(y1-y2) < TOL: hs.append((snap(y1), min(snap(x1),snap(x2)), max(snap(x1),snap(x2))))
            else: others.append(((x1,y1),(x2,y2)))
        else: others.append(((x1,y1),(x2,y2)))
    # collect candidate vertices
    xs = sorted({x for (x,_,_) in vs} | {x for (_,x0,x1) in hs for x in (x0,x1)})
    ys = sorted({y for (y,_,_) in hs} | {y for (_,y0,y1) in vs for y in (y0,y1)})
    hmap = {}   # (y,x0,x1)->True ; build per-y map
    vmap = {}
    for (y,x0,x1) in hs: hmap[(y,x0,x1)] = True
    for (x,y0,y1) in vs: vmap[(x,y0,y1)] = True
    # edges: coverage at (y,x) horizontal: membership sets
    rects = []
    # brute pairs of horizontal segs sharing x-span with verticals at ends
    hslist = list(hmap.keys())
    for i in range(len(hslist)):
        for j in range(i+1, len(hslist)):
            (y1,a1,b1),(y2,a2,b2) = hslist[i], hslist[j]
            if abs(y1-y2) < 0.01: continue
            x0, x1 = max(min(a1,b1), min(a2,b2)), min(max(a1,b1), max(a2,b2))
            if x1-x0 < 0.01: continue
            # corners must be covered by verticals
            ok = True
            for (cx, cy) in ((x0,y1),(x1,y1),(x0,y2),(x1,y2)):
                if not any(abs(cx-x) < 0.02 and min(yy0,yy1)-0.02 < cy < max(yy0,yy1)+0.02 for (x,yy0,yy1) in vmap):
                    ok = False; break
            if ok:
                rects.append([min(x0,x1), min(y1,y2), max(x0,x1), max(y1,y2)])
    return rects, hs, vs, others

for s in data:
    segs = s.get('segs')
    if not segs: continue
    rects, hs, vs, others = rects_from_segs(segs)
    print(f"== {s['sketch']}  (segs={len(segs)})")
    seen = set()
    for r in sorted(rects):
        k = tuple(snap(v) for v in r)
        if k in seen: continue
        seen.add(k)
        x0,y0,x1,y1 = k
        w, hh = round(x1-x0,3), round(y1-y0,3)
        print(f"    RECT ({x0:6.2f},{y0:6.2f})-({x1:6.2f},{y1:6.2f})  {w:5.2f}x{hh:5.2f}  cx={round((x0+x1)/2,2):6.2f} cy={round((y0+y1)/2,2):6.2f}")
    if others:
        print(f"    other-segs: {len(others)}", ['({:.1f},{:.1f})-({:.1f},{:.1f})'.format(*a,*b) for a,b in others][:10])
    if len(seen) != len(rects): pass
