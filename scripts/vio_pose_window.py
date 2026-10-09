"""Source-rate evidence uses ROS/sample time; retirement uses monotonic time."""


def source_window(poses,ros,boundary):
    return [p for p in poses if ros-5<=p['stamp']<=ros and p['mono']<boundary]
