"""Shared S0 artifact contracts. No ROS imports or simulation side effects."""
import hashlib
import json
import math
from pathlib import Path

import jsonschema
import yaml

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    with Path(path).open() as stream:
        return yaml.safe_load(stream)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate(value, schema_name):
    schema = read(ROOT / 'simulation/schema' / f'{schema_name}.schema.json')
    jsonschema.Draft202012Validator(schema).validate(value)
    # JSON Schema treats Python NaN/Infinity as numbers. JSON artifacts must not.
    json.dumps(value, allow_nan=False)


def load_acceptance(path):
    config = read(path)
    validate(config, 'acceptance')
    if any(not key.startswith('S0_') for key in config['tests']) and not config['metrics']:
        raise ValueError('INVALID_CONFIG: acceptance metrics not frozen')
    ids = [m['metric_id'] for m in config['metrics']]
    if len(ids) != len(set(ids)):
        raise ValueError('duplicate metric_id')
    for metric in config['metrics']:
        expected = digest({k: v for k, v in metric.items() if k != 'config_hash'})
        if metric['config_hash'] != expected:
            raise ValueError(f"stale config_hash: {metric['metric_id']}")
        if not set(metric['applicable_tests']) <= config['tests'].keys():
            raise ValueError('metric references undeclared test')
    return config


def evaluate(metric, samples):
    """Evaluate already-windowed observations; missing/nonfinite data never passes."""
    if not samples or any(isinstance(v, bool) or not isinstance(v, (int, float))
                          or not math.isfinite(v) for v in samples):
        return 'INCONCLUSIVE'
    op = metric['operator']
    threshold = metric['threshold']
    checks = {'<=': lambda v: v <= threshold, '<': lambda v: v < threshold,
              '>=': lambda v: v >= threshold, '>': lambda v: v > threshold,
              '==': lambda v: v == threshold}
    return 'PASS' if all(checks[op](v) for v in samples) else 'FAIL'


def overall(tests):
    required = [t['status'] for t in tests if t['required']]
    if not required:
        return 'INCONCLUSIVE'
    if 'FAIL' in required:
        return 'FAIL'
    return 'PASS' if all(s == 'PASS' for s in required) else 'INCONCLUSIVE'


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                    allow_nan=False) + '\n')
