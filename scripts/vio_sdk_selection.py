"""One explicit VIO default; missing or drifted SDK16 fails without fallback."""
import json
import os
from pathlib import Path
from build_vio_sdk_candidate import ROOT, COMMIT, sha

DEFAULT_SDK = 'sdk16'


def sdk16_runtime():
    from build_vio_sdk16_default import FOLDER, PREFIX, CONTRACT
    receipt = FOLDER/'ros-contract.json'
    if not receipt.is_file():
        raise RuntimeError('SDK16 is the default. Run ./scripts/build_vio_node.sh first.')
    contract = json.loads(receipt.read_text())
    if contract['contract'] != CONTRACT or contract['source_commit'] != COMMIT or contract['flight_authorized']:
        raise RuntimeError('Unexpected default SDK16 contract')
    for name,digest in {**contract['expected_sha256'],**contract['protected_baseline_sha256']}.items():
        if not (ROOT/name).is_file() or sha(ROOT/name) != digest:
            raise RuntimeError('SDK16 contract drift: '+name)
    return PREFIX, CONTRACT, contract


def sdk16_environment(base=None):
    prefix,contract,receipt = sdk16_runtime()
    env = dict(os.environ if base is None else base)
    env['AMENT_PREFIX_PATH'] = str(prefix)+':'+env.get('AMENT_PREFIX_PATH','')
    env['LD_LIBRARY_PATH'] = str(prefix/'lib')+':'+env.get('LD_LIBRARY_PATH','')
    return env
