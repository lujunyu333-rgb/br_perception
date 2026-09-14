# -*- coding: utf-8 -*-
import io, sys
sys.path.insert(0, r'C:\Users\希望\br_perception\.tmp_pdf')
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import win32com.client, pythoncom
pythoncom.CoInitialize()
from comutil import V
from win32com.client import gencache
mod = gencache.EnsureModule('{83A33D31-27C5-11CE-BFD4-00400513BB57}', 0, 33, 0)
dyn = win32com.client.GetActiveObject('SldWorks.Application')
d = (V(dyn, 'GetDocuments') or ())[0]
nd = mod.IModelDoc2(d._oleobj_)
print('RenderMaterial members on doc:', [a for a in dir(nd) if 'ender' in a][:10])
try:
    part = mod.PartDoc(d._oleobj_)
    bodies = part.GetBodies2(0, False)
    print('bodies:', type(bodies), len(bodies) if bodies else 0)
    if bodies:
        bb = mod.IBody2(bodies[0]._oleobj_)
        print('IBody2 sample:', [a for a in dir(bb) if any(k in a for k in ('Entity','Color','Appearance','Render','Visible','Feature'))][:20])
except Exception as e:
    print('PartDoc err:', str(e)[:120])
