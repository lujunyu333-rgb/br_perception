# -*- coding: utf-8 -*-
import json, io, sys
from collections import defaultdict
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
data = json.load(open(r'C:\Users\希望\br_perception\.tmp_pdf\sketch_segments.json', encoding='utf-8'))

def find_rects(segs, tol=0.002):
    # to field coords: (x_sw, -z_sw); keep height z0
    hs, vs = [], []  # hs: (y, x0, x1); vs: (x, y0, y1)
    points = []
    for s in segs:
        if s['t'] != 'L' or not s['p1'] or not s['p2']:
            continue
        (x1, y1, z1), (x2, y2, z2) = s['p1'], s['p2']
        h1 = round(y1, 4)
        # line in SW: on some plane. treat x1==x2 (vertical in x) vs y1==y2
        if abs(x1 - x2) < tol:
            vs.append((round(x1,4), round(z1,4), round(z2,4), h1))
        elif abs(z1 - z2) < tol:
            hs.append((round(z1,4), round(x1,4), round(x2,4), round(y1,4)))
        points.append((x1,y1,z1)); points.append((x2,y2,z2))
    # group heights
    hset = sorted({p[1] for p in points})
    # rect candidates: for each pair (y0,y1) of horizontal segs spanning same x, check verticals exist at both ends
    byH = defaultdict(list)
    for (y, a, b, h) in hs:
        byH[(min(a,b), max(a,b))].append(y)
    rects = []
    span_keys = list(byH.keys())
    for i in range(len(span_keys)):
        for j in range(i+1, len(span_keys)):
            kx, ky = span_keys[i], span_keys[j]
            x0, x1 = min(kx[0],ky[0]), max(kx[1],ky[1])
            xa, xb = kx[0], kx[1]
            xc, xd = ky[0], ky[1]
            if not (abs(xa-xc) < tol or abs(xa-xd) < tol or abs(xb-xc) < tol or abs(xb-xd) < tol):
                continue
            for ya in byH[kx]:
                for yb in byH[ky]:
                    if abs(ya-yb) < tol: continue
                    # need vertical edges at the matching shared x
                    # quick accept: both spans share an endpoint x within tol
                    xs = sorted({xa, xb, xc, xd})
                    xs = [x for x in xs if xs.count(x)] 
                    if len(set([round(xa,3),round(xb,3),round(xc,3),round(xd,3)])) < 4:
                        rects.append([min(xa,xb,xc,xd), min(ya,yb), max(xa,xb,xc,xd), max(ya,yb)])
    return rects

for s in data:
    if 'error' in s or not s['segs']:
        print(s.get('sketch'), s.get('error', 'no segs')); continue
    pts = [p for g in s['segs'] if g['p1'] for p in (g['p1'], g['p2'])]
    heights = sorted({round(p[1],3) for p in pts})
    rects = find_rects(s['segs'])
    if not rects: continue
    # dedupe
    seen = set(); uniq = []
    for r in rects:
        k = tuple(round(v,3) for v in r)
        if k not in seen: seen.add(k); uniq.append(r)
    print(f"--- {s['sketch']}: segs={len(s['segs'])} sw-y(heights)={heights}")
    for r in sorted(uniq)[:40]:
        x0,y0,x1,y1 = [round(v,3) for v in r]
        print(f"    rect x[{x0},{x1}] y[{y0},{y1}] size {round(x1-x0,3)}x{round(y1-y0,3)} center ({round((x0+x1)/2,3)},{round((y0+y1)/2,3)})")
