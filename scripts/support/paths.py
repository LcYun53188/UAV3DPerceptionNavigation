"""Locate workspace tool modules without flattening their files into scripts/."""
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
LAYOUT = SCRIPTS / 'compat/layout.json'


def enable_imports():
    """Keep historical module names usable from canonical entrypoints."""
    layout = json.loads(LAYOUT.read_text())
    directories = {SCRIPTS}
    directories.update((ROOT / target).parent for target in layout['scripts'].values()
                       if target.endswith('.py'))
    missing = [str(path) for path in sorted(directories) if str(path) not in sys.path]
    sys.path[:0] = missing
