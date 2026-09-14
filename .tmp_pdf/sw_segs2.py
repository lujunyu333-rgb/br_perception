# -*- coding: utf-8 -*-
import win32com.client, pythoncom, io, sys, json
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
pythoncom.CoInitialize()
from win32com.client import gencache
mod = gencache.EnsureModule('{83A33D31-27C5-11CE-BFD4-00400513BB57}', 0, 33, 0)
dyn = win32com.client.GetActiveObject('SldWorks.Application')
d = dyn.GetDocuments()[0]
nd = mod.IModelDoc2(d._oleobj_)
fm = nd.FeatureManager
feats = fm.GetFeatures(False)

def coords(pt):
    try:
        c = pt.GetCoords()
        return [round(float(v), 4) for v in c]
    except Exception:
        return None

classes = [(n, getattr(mod, n)) for n in ('ISketchLine','ISketchArc','ISketchEllipse') if hasattr(mod, n)]

def seg_endpoints(elem):
    for cname, cls in classes:
        try:
            s = cls(elem._oleobj_)
            p1 = s.GetStartPoint2() if hasattr(s, 'GetStartPoint2') else None
            p2 = s.GetEndPoint2() if hasattr(s, 'GetEndPoint2') else None
            return cname, (coords(p1) if p1 else None), (coords(p2) if p2 else None)
        except Exception:
            continue
    return None, None, None

out = []
for fe in feats:
    tf = mod.IFeature(fe._oleobj_)
    if tf.GetTypeName2() != 'ProfileFeature':
        continue
    sp = tf.GetSpecificFeature2()   # dynamic
    try:
        segs = sp.GetSketchSegments
    except Exception:
        out.append({'sketch': tf.Name, 'error': 'no-segs'})
        continue
    if not isinstance(segs, (tuple, list)):
        out.append({'sketch': tf.Name, 'error': repr(segs)[:60]})
        continue
    lines, nOther = [], 0
    for el in segs:
        cname, c1, c2 = seg_endpoints(el)
        if cname == 'ISketchLine':
            lines.append({'t': 'L', 'p1': c1, 'p2': c2})
        else:
            lines.append({'t': (cname or '?')[:12], 'p1': c1, 'p2': c2})
            if not cname: nOther += 1
    out.append({'sketch': tf.Name, 'segs': lines})

with open(r'C:\Users\希望\br_perception\.tmp_pdf\sketch_segments.json', 'w', encoding='utf-8') as f:
    json.dump(out, f, ensure_ascii=False)
tot = sum(len(s['segs']) for s in out if 'segs' in s)
kinds = {}
for s in out:
    for g in s.get('segs', []): kinds[g['t']] = kinds.get(g['t'], 0) + 1
print('sketches:', len(out), 'segs:', tot, 'kinds:', kinds)
