"""Keep automatic cyclic GC outside a bounded owned observation/control window."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import gc
import time


class ObservationGcGuard:
    def __init__(self, enabled):
        self.originally_enabled = gc.isenabled()
        self.active = bool(enabled)
        self.entered_at = time.monotonic()
        self.restored_at = None
        if self.active:
            gc.disable()

    def restore(self):
        if self.restored_at is not None:
            return
        if self.active:
            if self.originally_enabled:
                gc.enable()
            else:
                gc.disable()
        self.restored_at = time.monotonic()

    def receipt(self):
        return dict(schema=1, active=self.active, originally_enabled=self.originally_enabled,
                    entered_mono=self.entered_at, restored_mono=self.restored_at,
                    restored_enabled=gc.isenabled(),
                    scope='automatic cyclic GC deferred during bounded owned observation/control; watchdog and original sample times unchanged')
