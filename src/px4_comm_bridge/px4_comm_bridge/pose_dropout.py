"""Reject individual uncertain poses within the existing last-good deadline."""


def can_drop(policy,reason,*,bound,ros,mono,last_sample,last_receive):
    return (policy=='bounded_gap' and reason=='VIO_UNCERTAINTY_INVALID' and bound
        and last_sample is not None and last_receive is not None
        and -.05<=ros-last_sample<=.2 and 0<=mono-last_receive<=.2)
