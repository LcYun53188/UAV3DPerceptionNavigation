"""Give the profiled SDK its own session so SIGINT cannot cancel Nsight export."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import json
import os
from pathlib import Path
import sys


def identity(pid):
    fields=Path('/proc/'+str(pid)+'/stat').read_text().rsplit(')',1)[1].split()
    return dict(pid=pid,pgid=int(fields[2]),sid=int(fields[3]),start_ticks=int(fields[19]))


def main():
    receipt=Path(sys.argv[1]);command=sys.argv[2:]
    if os.getpgrp()==os.getpid() and os.getsid(0)!=os.getpid():
        child=os.fork()
        if child:
            _,status=os.waitpid(child,0)
            return os.waitstatus_to_exitcode(status)
    if os.getsid(0)!=os.getpid():os.setsid()
    receipt.write_text(json.dumps(identity(os.getpid()))+'\n')
    os.execvp(command[0],command)


if __name__=='__main__':sys.exit(main())
