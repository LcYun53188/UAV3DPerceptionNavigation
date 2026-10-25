"""Bounded last-observed places, separate from live person tracks and identity."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class HistoricalPerson:
    id: int
    source_track_id: str
    position: np.ndarray
    stamp: float
    confidence: float


class PersonHistory:
    def __init__(self, radius=.6, max_records=200):
        if not math.isfinite(radius) or radius <= 0 or type(max_records) is not int or max_records < 1:
            raise ValueError('Invalid person history limits')
        self.radius, self.max_records = radius, max_records
        self.records = {}
        self.next_id = 1

    def clear(self):
        self.records.clear()

    def update(self, observations):
        """Accept confirmed, valid map observations; never infer current presence."""
        observations = list(observations)
        if any(not source or not np.isfinite(position).all() or np.shape(position) != (3,)
               or not math.isfinite(stamp) or stamp <= 0 or not 0 <= confidence <= 1
               for source, position, stamp, confidence in observations):
            raise ValueError('Invalid historical person observation')
        pairs = sorted((record.source_track_id != source,
                        float(np.linalg.norm(record.position-position)), key, index)
                       for key, record in self.records.items()
                       for index, (source, position, _, _) in enumerate(observations)
                       if np.linalg.norm(record.position-position) <= self.radius)
        used_records, used_inputs = set(), set()
        for _, _, key, index in pairs:
            if key in used_records or index in used_inputs:
                continue
            source, position, stamp, confidence = observations[index]
            record = self.records[key]
            if stamp > record.stamp:
                record.source_track_id, record.position = source, np.array(position, dtype=float, copy=True)
                record.stamp, record.confidence = stamp, confidence
            used_records.add(key)
            used_inputs.add(index)
        for index, (source, position, stamp, confidence) in enumerate(observations):
            if index in used_inputs:
                continue
            # New places consume a fixed budget; evict the least recent record.
            if len(self.records) >= self.max_records:
                oldest = min(self.records, key=lambda key: (self.records[key].stamp, key))
                del self.records[oldest]
            key = self.next_id
            self.next_id += 1
            self.records[key] = HistoricalPerson(key, source, np.array(position, dtype=float, copy=True), stamp, confidence)
