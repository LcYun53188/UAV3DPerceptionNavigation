"""Give the profiled SDK its own session so SIGINT cannot cancel Nsight export."""
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
