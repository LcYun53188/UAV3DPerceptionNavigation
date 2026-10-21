"""Bounded receive/source timing evidence; never substitutes or validates input."""
from collections import deque


class ManualInputEvidence:
    def __init__(self):
        self.samples = self.invalid_samples = self.non_increasing_stamps = 0
        self.last_receive = self.last_stamp = self.last_valid = None
        self.max_receive_gap_s = self.max_source_gap_s = 0.
        self.events = deque(maxlen=64)

    def receive(self, message, now):
        stamp = int(message.timestamp)
        receive_gap = 0. if self.last_receive is None else now-self.last_receive
        source_gap = 0. if self.last_stamp is None else (stamp-self.last_stamp)/1e6
        self.max_receive_gap_s = max(self.max_receive_gap_s,receive_gap)
        self.max_source_gap_s = max(self.max_source_gap_s,source_gap)
        valid = bool(message.valid)
        self.samples += 1
        self.invalid_samples += not valid
        repeated = self.last_stamp is not None and stamp <= self.last_stamp
        self.non_increasing_stamps += repeated
        if valid != self.last_valid or repeated or receive_gap > .2 or source_gap > .2:
            self.events.append(dict(mono=now,native_timestamp_us=stamp,valid=valid,
                data_source=int(message.data_source),receive_gap_s=receive_gap,
                source_gap_s=source_gap,non_increasing_stamp=repeated))
        self.last_receive,self.last_stamp,self.last_valid = now,stamp,valid

    def snapshot(self, now):
        return dict(samples=self.samples,invalid_samples=self.invalid_samples,
            non_increasing_stamps=self.non_increasing_stamps,
            receive_age_s=None if self.last_receive is None else max(0.,now-self.last_receive),
            max_receive_gap_s=self.max_receive_gap_s,max_source_gap_s=self.max_source_gap_s,
            events=list(self.events),scope='native PX4 messages received by gateway; no QGC send or USB continuity inference')
