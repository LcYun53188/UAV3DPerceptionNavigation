import math
from px4_camera_mount_audit import audit


def scene(rig_z):
    return '''model {name: "x500_depth_ref_7"
      link {name: "base_link" pose {position {z: .24} orientation {w: 1}}}
      link {name: "vio_rig_link" pose {position {x: .12 z: %s} orientation {w: 1}}
        sensor {name: "mapping_rgbd" pose {position {} orientation {y: %.15f w: %.15f}}}}
    }''' % (rig_z,math.sin(math.radians(5)/2),math.cos(math.radians(5)/2))


def frames():return {'oakd_camera_optical_frame':dict(position=[.12,0,.242],rpy=[-math.pi/2-math.radians(5),0,-math.pi/2])}


def test_model_relative_mount_rejected_and_body_relative_mount_passes():
    old=audit(scene(.242),'x500_depth_ref_7',frames())
    assert not old['passed'] and abs(old['translation_error_m']-.24)<1e-12
    new=audit(scene(.482),'x500_depth_ref_7',frames())
    assert new['passed'] and new['rotation_error_rad']<1e-6


def test_missing_duplicated_or_rotated_mount_rejected():
    assert not audit(scene(.482)*2,'x500_depth_ref_7',frames())['passed']
    assert not audit('', 'x500_depth_ref_7',frames())['passed']
    bad=frames();bad['oakd_camera_optical_frame']['rpy'][0]+=math.radians(5)
    assert not audit(scene(.482),'x500_depth_ref_7',bad)['passed']
