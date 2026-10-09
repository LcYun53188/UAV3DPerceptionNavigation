"""Explicit renderer selection for owned Gazebo only; never changes machine settings."""
import hashlib
import re
from pathlib import Path


def environment(base,device):
    if device not in ('default','nvidia'):raise ValueError('Unknown render device')
    result=dict(base)
    if device=='nvidia':
        result.update(__NV_PRIME_RENDER_OFFLOAD='1',__GLX_VENDOR_LIBRARY_NAME='nvidia')
    return result


def capture(path,started_system_ns,requested):
    path=Path(path)
    if not path.is_file():return dict(passed=False,reason='RENDER_LOG_MISSING')
    raw=path.read_bytes();content=raw.decode(errors='replace')
    vendors=re.findall(r'GL_VENDOR = (.+)',content);renderers=re.findall(r'GL_RENDERER = (.+)',content)
    fresh=path.stat().st_mtime_ns>=started_system_ns
    matched=bool(vendors and renderers) and (requested!='nvidia' or 'NVIDIA' in vendors[-1])
    return dict(passed=fresh and matched,requested=requested,fresh_log=fresh,
        vendor=vendors[-1] if vendors else None,renderer=renderers[-1] if renderers else None,
        source=str(path),mtime_ns=path.stat().st_mtime_ns,sha256=hashlib.sha256(raw).hexdigest())
