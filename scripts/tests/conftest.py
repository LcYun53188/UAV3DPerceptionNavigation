"""Shared tool imports for versioned checks and ignored local regressions."""
from pathlib import Path
import sys


SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(SCRIPTS / 'tests/local'))
from support.paths import enable_imports

enable_imports()
