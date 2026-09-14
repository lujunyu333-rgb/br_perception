# -*- coding: utf-8 -*-
import win32com.client, pythoncom
def V(obj, name, *args, **kw):
    """value-or-call helper for flaky SW dynamic dispatch"""
    try:
        v = getattr(obj, name)
    except Exception:
        return None
    if callable(v):
        try:
            return v(*args, **kw)
        except Exception:
            return None
    return v
