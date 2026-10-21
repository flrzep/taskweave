"""The offline paths (``taskweave check --missions``, ``describe``) and the move schema need no ROS node: they run
in a process where ``rclpy`` cannot be imported (a fresh interpreter, so no module is cached from a test that has it)."""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

GOLDEN = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures' / 'migrate_targets' / 'docs_cell'

NO_RCLPY = 'import sys\nsys.modules["rclpy"] = None   # import rclpy raises ImportError\n'


def run_without_rclpy(code: str, *args: str) -> str:
    env = {**os.environ, 'PYTHONPATH': os.pathsep.join(p for p in sys.path if p), 'PYTHONDONTWRITEBYTECODE': '1'}
    done = subprocess.run([sys.executable, '-c', NO_RCLPY + code, *args], env=env, capture_output=True, text=True,
                          timeout=120)
    assert done.returncode == 0, done.stderr[-2000:]
    return done.stdout


def test_the_move_class_and_its_schema_load_without_rclpy():
    out = run_without_rclpy('''
import json, sys
from taskweave_core.skill import load_skill_class
cls = load_skill_class('taskweave_skills.move:Move')
schema = cls.schema_for(['left', 'right'])
print(json.dumps({'name': cls.name, 'server': 'taskweave_core.skill_server' in sys.modules,
                  'hand': schema['properties']['hand']['anyOf'][0]['enum']}))
''')
    assert json.loads(out) == {'name': 'move', 'server': False, 'hand': ['all', 'left', 'right']}


def test_check_missions_and_the_offline_catalog_run_without_rclpy(tmp_path):
    pytest.importorskip('moveit_msgs')     # the other skill classes of the cell import their message types
    pytest.importorskip('sensor_msgs')
    cell = tmp_path / 'cell'
    import shutil
    shutil.copytree(GOLDEN, cell)
    out = run_without_rclpy('''
import json, sys
from taskweave_core import cli
from taskweave_core.config import ConfigStore, packs
packs.installed_location = lambda: None
store = ConfigStore(sys.argv[1])
catalog = cli.offline_catalog(store, 'duo_bot')
code = cli.check_missions(sys.argv[1])
print(json.dumps({'skills': sorted(catalog), 'server': 'taskweave_core.skill_server' in sys.modules, 'code': code}))
''', str(cell))
    result = json.loads(out.strip().splitlines()[-1])
    assert result == {'skills': ['grip', 'move', 'wait'], 'server': False, 'code': 0}, out
