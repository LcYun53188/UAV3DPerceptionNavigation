"""Original work saved before a pilot return; no trajectory or flight commands."""
from copy import deepcopy
from dataclasses import dataclass


@dataclass(frozen=True)
class ReturnCheckpoint:
    steps: tuple
    next_index: int
    hover_remaining: float | None
    hover_target: tuple | None
    root: bytes
    coordinator: str
    session: bytes
    home: tuple
    deadline: float
    resets: tuple
    alignment: tuple
    map_context: tuple | None

    def restore_steps(self):
        # Each revision gets new data, never an old Segment/child/planned curve.
        return deepcopy(list(self.steps))
