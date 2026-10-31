"""Safety expectations replacing the audit's pre-fix counterexamples."""
from collections import deque
from copy import deepcopy
from pathlib import Path
import json
import sys
import time
from types import SimpleNamespace as NS

import pytest
pytest.importorskip('rclpy')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'simulation/vio/runtime'))
from person_follow_backend import PersonFollowBackend
from person_follow_client import PersonFollowClient
from person_follow_core import FollowPreview
from vision_monitor import VisionMonitor


@pytest.mark.parametrize('value',[123,None,[],{}])
def test_wrong_uuid_type_is_rejected_by_receivers(value):
    PersonFollowBackend.select(NS(),NS(data=json.dumps({'client_id':value})))
    PersonFollowClient.receive(NS(),NS(data=json.dumps({'coordinator_instance':value,'mission_uuid':'unused'})))


def test_same_stamp_invalidation_revokes_preview_without_renewing_receipt():
    core=FollowPreview()
    core.observe('person',[0,0,1],[-5,0,2],10.)
    core.command('START')
    context=NS(header=NS(stamp=NS(sec=10,nanosec=0)),valid=True)
    backend=NS(cache={'context':(context,123.)},core=core,context_key=('old',))
    invalid=deepcopy(context)
    invalid.valid=False
    PersonFollowBackend.receive(backend,'context',invalid)
    assert core.state=='LOST' and not backend.cache['context'][0].valid
    assert backend.cache['context'][1]==123.


def test_repeated_capture_with_different_point_is_rejected():
    core=FollowPreview()
    core.observe('person',[0,0,1],[-5,0,2],10.)
    core.command('START')
    core.observe('person',[100,0,1],[-5,0,2],10.)
    assert core.state=='LOST' and core.goal is None


def test_old_detection_arrival_does_not_rewind_the_display():
    def message(stamp):
        return NS(header=NS(stamp=NS(sec=10,nanosec=stamp),frame_id='camera'))
    older,newer=message(0),message(100000000)
    mono=time.monotonic()
    view=NS(node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=10200000000))),
        rgb_frames=deque([(older,mono),(newer,mono)]),detections=deque([(newer,mono),(older,mono)]),
        image_serial={'rgb':('camera',10.1)})
    assert VisionMonitor.display_rgb(view) is newer
