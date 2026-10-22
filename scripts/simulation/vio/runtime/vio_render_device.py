"""Explicit renderer selection for owned Gazebo only; never changes machine settings."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import hashlib
import re
from pathlib import Path


def environment(base,device):
    if device not in ('default','nvidia','intel'):raise ValueError('Unknown render device')
    result=dict(base)
    if device=='nvidia':
        result.update(__NV_PRIME_RENDER_OFFLOAD='1',__GLX_VENDOR_LIBRARY_NAME='nvidia')
    elif device=='intel':
        mesa=Path('/usr/share/glvnd/egl_vendor.d/50_mesa.json')
        if not mesa.is_file():raise ValueError('Mesa EGL vendor configuration missing')
        result.pop('__NV_PRIME_RENDER_OFFLOAD',None)
        result.pop('__NV_PRIME_RENDER_OFFLOAD_PROVIDER',None)
        result.update(__GLX_VENDOR_LIBRARY_NAME='mesa',DRI_PRIME='0',
            __EGL_VENDOR_LIBRARY_FILENAMES=str(mesa))
    return result


def capture(path,started_system_ns,requested):
    path=Path(path)
    if not path.is_file():return dict(passed=False,reason='RENDER_LOG_MISSING')
    raw=path.read_bytes();content=raw.decode(errors='replace')
    vendors=re.findall(r'GL_VENDOR = (.+)',content);renderers=re.findall(r'GL_RENDERER = (.+)',content)
    fresh=path.stat().st_mtime_ns>=started_system_ns
    matched=bool(vendors and renderers) and (requested!='nvidia' or 'NVIDIA' in vendors[-1])
    if requested=='intel':
        matched=bool(vendors and renderers and 'Intel' in renderers[-1]
            and not any(s in renderers[-1].lower() for s in ('llvmpipe','softpipe','software')))
    return dict(passed=fresh and matched,requested=requested,fresh_log=fresh,
        vendor=vendors[-1] if vendors else None,renderer=renderers[-1] if renderers else None,
        source=str(path),mtime_ns=path.stat().st_mtime_ns,sha256=hashlib.sha256(raw).hexdigest())
