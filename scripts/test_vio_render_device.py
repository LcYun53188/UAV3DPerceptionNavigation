from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from vio_render_device import environment,capture


def test_selection_changes_only_copied_environment():
    base={'ROS_DOMAIN_ID':'78'};selected=environment(base,'nvidia')
    assert base=={'ROS_DOMAIN_ID':'78'}
    assert selected['__NV_PRIME_RENDER_OFFLOAD']=='1' and selected['__GLX_VENDOR_LIBRARY_NAME']=='nvidia'
    assert environment(base,'default')==base


def test_requested_gpu_requires_fresh_actual_vendor(tmp_path):
    p=tmp_path/'ogre.log';p.write_text('GL_VENDOR = Intel\nGL_RENDERER = Mesa Intel\n')
    assert not capture(p,0,'nvidia')['passed']
    p.write_text('GL_VENDOR = NVIDIA Corporation\nGL_RENDERER = NVIDIA RTX\n')
    assert capture(p,0,'nvidia')['passed']
    assert not capture(p,p.stat().st_mtime_ns+1,'nvidia')['passed']
    assert not capture(tmp_path/'missing',0,'nvidia')['passed']
