# -*- coding: utf-8 -*-
import win32com.client, pythoncom, io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
pythoncom.CoInitialize()
from win32com.client import gencache
mod = gencache.EnsureModule('{83A33D31-27C5-11CE-BFD4-00400513BB57}', 0, 33, 0)
dyn = win32com.client.GetActiveObject('SldWorks.Application')
d = dyn.GetDocuments()[0]
nd = mod.IModelDoc2(d._oleobj_)
fm = nd.FeatureManager
feats = fm.GetFeatures(False)
tf = None
for fe in feats:
    x = mod.IFeature(fe._oleobj_)
    if x.GetTypeName2() in ('ICE','ProfileFeature'):
        tf = x
        print('test on:', x.GetTypeName2(), x.Name)
        for opt in (0,1,2):
            try:
                v = tf.GetBox(opt)
                print('  opt',opt,'->', type(v), repr(v)[:200])
            except Exception as e:
                print('  opt',opt,'ERR', str(e)[:80])
        break
