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

def seg_endpoints(elem, classes):
    # try known segment interfaces in order
    for cname, cls in classes:
        try:
            s = cls(elem._oleobj_)
            t = None
            p1 = s.GetStartPoint2() if hasattr(s, 'GetStartPoint2') else None
            p2 = s.GetEndPoint2() if hasattr(s, 'GetEndPoint2') else None
            c1 = coords(p1) if p1 else None
            c2 = coords(p2) if p2 else None
            return cname, c1, c2
        except Exception:
            continue
    return None, None, None

classes = []
for n in ('ISketchLine','ISketchArc','ISketchEllipse','ISketchParabola'):
    if hasattr(mod, n):
        classes.append((n, getattr(mod, n)))

out = []
for fe in feats:
    tf = mod.IFeature(fe._oleobj_)
    if tf.GetTypeName2() != 'ProfileFeature':
        continue
    try:
        sp = mod.ISketch(tf.GetSpecificFeature2()._oleobj_)
        segs = sp.GetSketchSegments()
    except Exception as e:
        out.append({'sketch': tf.Name, 'error': str(e)[:80]})
        continue
    lines = []
    nline = nOther = 0
    for el in segs:
        cname, c1, c2 = seg_endpoints(el, classes)
        if cname == 'ISketchLine':
            lines.append({'t': 'L', 'p1': c1, 'p2': c2})
            nline += 1
        elif cname:
            lines.append({'t': 'A', 'p1': c1, 'p2': c2})
            nOther += 1
        else:
            nOther += 1
    out.append({'sketch': tf.Name, 'lines': nline, 'other': nOther, 'segs': lines})

with open(r'C:\Users\希望\br_perception\.tmp_pdf\sketch_segments.json', 'w', encoding='utf-8') as f:
    json.dump(out, f, ensure_ascii=False)
print('ok, sketches:', len(out), '| total segs:', sum(s['lines']+s['other'] for s in out if 'lines' in s))
