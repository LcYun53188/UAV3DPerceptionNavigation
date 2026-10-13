"""Keep automatic cyclic GC outside a bounded owned observation/control window."""
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
