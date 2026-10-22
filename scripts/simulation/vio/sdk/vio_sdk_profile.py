"""Optional CUDA/NVTX trace; profiling is never source qualification."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

from pathlib import Path
import os
import signal
import shutil
import subprocess
import time
import json
import sys


def command(binary_command, output):
    executable=shutil.which('nsys')
    if executable is None:raise RuntimeError('Nsight Systems is not installed')
    return [executable,'profile','--trace=cuda,nvtx','--sample=none',
        '--cpuctxsw=none','--resolve-symbols=false','--force-overwrite=true','--wait=all',
        '--session-new='+session_name(output),
        '--output='+str(Path(output)),sys.executable,
        str(Path(__file__).with_name('vio_sdk_target.py')),
        str(output)+'.target.json']+list(binary_command)


def session_name(output):
    return 'vio-'+Path(output).parent.name


def signal_target(output, signum):
    from vio_sdk_target import identity
    target=json.loads(Path(str(output)+'.target.json').read_text())
    try:actual=identity(target['pid'])
    except FileNotFoundError:return True
    if target!=actual or target['pgid']!=target['pid'] or target['sid']!=target['pid']:
        raise ValueError('SDK_PROFILE_TARGET_IDENTITY_CHANGED')
    try:os.killpg(target['pgid'],signum)
    except ProcessLookupError:pass
    return False


def stop_profile(process, output):
    """Allow export to finish; check the named session outside the root group."""
    from run_px4_sitl_smoke import stop
    result=dict(session=session_name(output),wrapper_exited=False,
        session_closed=False,target_closed=False,report_present=False,error=None)
    deadline=time.monotonic()+45
    try:
        signal_target(output,signal.SIGINT)
        try:process.wait(timeout=40)
        except subprocess.TimeoutExpired:
            # Shut down only this run's session, never all profiler sessions.
            subprocess.run(['nsys','shutdown','--session='+result['session'],'--kill=none'],
                capture_output=True,text=True,timeout=5,check=True)
            process.wait(timeout=5)
        result['wrapper_exited']=process.poll() is not None
        # The importer can outlive the CLI root. Do not interrupt it merely
        # because wait() returned; an exported report is the completion receipt.
        while not Path(str(output)+'.nsys-rep').is_file() and time.monotonic()<deadline:
            time.sleep(.1)
        sessions=subprocess.run(['nsys','sessions','list'],capture_output=True,
            text=True,timeout=5,check=True).stdout
        result['session_closed']=not any(result['session'] in row.split() for row in sessions.splitlines())
    except (OSError,ValueError,KeyError,subprocess.SubprocessError) as exc:
        result['error']=str(exc)
    finally:
        try:
            result['target_closed']=signal_target(output,signal.SIGKILL)
        except (OSError,ValueError,KeyError) as exc:
            result['error']=str(exc)
        stop(process)
    result['report_present']=Path(str(output)+'.nsys-rep').is_file()
    result['cleanup_confirmed']=result['wrapper_exited'] and result['session_closed'] and result['target_closed'] and result['error'] is None
    return result
