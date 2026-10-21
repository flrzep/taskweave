"""The robot packs of taskweave_robots against a small cell: every pack has a valid pack.yaml, its files validate, and a
generic mission (drive, a named pose, the gripper) validates for each robot. Cell content (a pose or an envelope made
for one cell's objects) is not in a robot pack: a mission that needs it validates only when the cell pack provides it.

The packs are read from this source folder (the parent of test/), not from an installed share, so the tests check
the packs as they are in the repo and run without the package built. They need taskweave_core and taskweave_skills
on the path (the skills' argument schemas are the catalog).
"""

from pathlib import Path

import pytest
import yaml

from taskweave_core.config import ConfigStore, packs
from taskweave_core.config.limits import pose_problems
from taskweave_core.embodiment import check_profile
from taskweave_core.plan import Plan
from taskweave_core.validation import validate_plan

ROBOTS = Path(__file__).resolve().parents[1]
PACKS = sorted(p.parent.name for p in ROBOTS.glob('*/pack.yaml'))
PACK_CONTENT = set(packs.ROBOT_PACK_FILES) | {packs.PACK_FILE}
NOTE = 'LICENSE-NOTE.md'                  # the licence note every pack carries (vendor-derived data)


def pack_folders(root: Path) -> list[str]:
    """The folders of ``root`` that are (meant to be) robot packs. Not these: hidden folders (.pytest_cache, .git),
    test/ and __pycache__, or a folder that holds neither a pack.yaml nor any robot pack file (a cache, a leftover).
    A folder with a profile.yaml but no pack.yaml stays in: that is a pack nobody finished."""
    return sorted(p.name for p in root.iterdir() if p.is_dir() and not p.name.startswith('.')
                  and p.name not in ('test', '__pycache__') and any((p / f).exists() for f in PACK_CONTENT))


FOLDERS = pack_folders(ROBOTS)

CELL_POSE = 'cell_grasp_approach'   # stands for a pose made for one cell's objects (cell pack, not robot pack)
CARRY = {'front': 1.2, 'back': 0.3, 'left': 0.5, 'right': 0.6}


@pytest.fixture(autouse=True)
def only_the_source_packs(monkeypatch):
    """The packs of this folder only: no installed share, no other env packs."""
    monkeypatch.setenv(packs.ROBOT_PACKS_ENV, str(ROBOTS))
    monkeypatch.setattr(packs, 'installed_location', lambda: None)


@pytest.fixture
def cell(tmp_path):
    """A small cell pack: one location, no robot files of its own."""
    root = tmp_path / 'cell'
    (root / 'cell').mkdir(parents=True)
    (root / 'missions').mkdir()
    (root / 'pack.yaml').write_text(f'name: test_cell\nversion: 1\nschema_version: {packs.CELL_SCHEMA}\n')
    (root / 'cell' / 'targets.yaml').write_text('waiting: {moves: base, x: 1.0, y: 0.5, yaw: 0.0}\n')
    (root / 'cell' / 'objects.yaml').write_text('{}\n')
    return root


def catalog(store: ConfigStore, robot: str) -> dict:
    """Argument schemas of the skills the generic mission uses, as the skill server would offer them (the hands
    narrowed to the robot's grippers, as the skills' setup does)."""
    import copy

    from taskweave_core.config.schemas import hands_schema
    from taskweave_skills.gripper_motion import Grip
    from taskweave_skills.move import Move

    hands = list(store.robot(robot).data.get('grippers', {}))
    grip = copy.deepcopy(Grip.args_schema)
    grip['properties']['hands'] = hands_schema(hands, ('auto', 'all'))
    return {Move.name: Move.schema_for(hands), Grip.name: grip}


def mission_plan(store: ConfigStore, robot: str, pose: str, tools: bool = False) -> Plan:
    """Drive, take a named target (a joint or a hand target: ``move to`` takes either; ``tools`` says it is a hand
    target and only documents the call), open a gripper."""
    hands = list(store.robot(robot).data.get('grippers', {}))[:1] or 'all'    # one gripper, else the rule
    mission = {'goal': 'generic', 'robot': robot, 'loop': 'once', 'steps': [
        {'id': 'drive', 'instruction': 'Drive to the waiting place', 'skill': 'move', 'args': {'to': 'waiting'}},
        {'id': 'pose', 'instruction': f'Take the {pose} pose', 'skill': 'move', 'args': {'to': pose}},
        {'id': 'open', 'instruction': 'Open a gripper', 'skill': 'grip', 'args': {'hands': hands, 'action': 'open'}},
    ]}
    (store.root / 'missions' / 'generic.yaml').write_text(yaml.safe_dump(mission, sort_keys=False))
    return Plan.from_mission(store.mission('generic'), 'test-0001')


def test_the_three_robots_have_packs():
    assert {'galbot_s1', 'galbot_g1', 'a2d'} <= set(PACKS)


@pytest.mark.parametrize('folder', FOLDERS)
def test_every_pack_has_a_valid_pack_yaml(folder):
    pack = packs.read_pack(ROBOTS / folder)    # validated against schemas.PACK
    assert pack is not None, f'{folder}/ has no pack.yaml (CMakeLists installs only folders with one)'
    assert pack['name'] == folder and pack['schema_version'] == packs.ROBOT_SCHEMA and pack.get('description')


def test_stray_folders_are_not_packs(tmp_path):
    """A cache or a build leftover next to the packs (.pytest_cache, .git, __pycache__, an empty folder) must not
    fail the suite; a folder with robot files but no pack.yaml is an unfinished pack and still counts."""
    for name in ('.pytest_cache', '.git', '__pycache__', 'test', 'leftover'):
        (tmp_path / name).mkdir()
    (tmp_path / 'leftover' / 'notes.txt').write_text('x')
    (tmp_path / 'good').mkdir()
    (tmp_path / 'good' / 'pack.yaml').write_text('name: good\n')
    (tmp_path / 'unfinished').mkdir()
    (tmp_path / 'unfinished' / 'profile.yaml').write_text('name: unfinished\n')
    (tmp_path / 'a_file.txt').write_text('x')
    assert pack_folders(tmp_path) == ['good', 'unfinished']


@pytest.mark.parametrize('robot', PACKS)
def test_every_pack_has_a_licence_note(robot):
    assert (ROBOTS / robot / NOTE).is_file(), f'{robot}/{NOTE} missing (vendor-derived data: internal use only)'


@pytest.mark.parametrize('robot', PACKS)
def test_a_pack_holds_only_robot_files(robot):
    extra = {p.name for p in (ROBOTS / robot).iterdir()} - PACK_CONTENT - {NOTE}
    assert not extra, f'{robot}: not robot pack files: {sorted(extra)} (cell-specific files belong to the cell pack)'


@pytest.mark.parametrize('robot', PACKS)
def test_profile_embodiment_and_robot_files_validate(cell, robot):
    store = ConfigStore(cell)
    assert store.robot_file_source(robot, 'profile.yaml')['source'] == 'robot pack'
    profile, embodiment = store.robot(robot), store.embodiment(robot)   # each validated against its schema
    assert profile.name == robot and embodiment['name'] == robot
    assert check_profile(embodiment, profile.data) == []
    store.navigation(robot)
    for name in store.joint_poses(robot):
        assert pose_problems(store, robot, name) == [], f'{robot} pose {name}'
    for file in ('sim.yaml', 'pose_targets.yaml'):     # the application's inputs: no taskweave schema, but YAML
        path = ROBOTS / robot / file
        assert not path.exists() or isinstance(yaml.safe_load(path.read_text(encoding='utf-8')), dict), file


@pytest.mark.parametrize('robot', PACKS)
def test_a_generic_mission_validates(cell, robot):
    store = ConfigStore(cell)
    assert 'home' in store.joint_poses(robot), f'{robot}: no home pose'
    assert validate_plan(mission_plan(store, robot, 'home'), catalog(store, robot), store) == []


@pytest.mark.parametrize('robot', PACKS)
def test_robot_packs_carry_no_cell_content(cell, robot):
    """Envelopes and poses sized for one cell's objects belong to the cell pack (robots/<robot>/ there)."""
    store = ConfigStore(cell)
    assert 'carry_envelope' not in store.navigation(robot)


def test_a_cell_pose_validates_only_when_the_cell_pack_provides_it(cell):
    robot = 'galbot_s1'
    store = ConfigStore(cell)
    problems = validate_plan(mission_plan(store, robot, CELL_POSE, tools=True), catalog(store, robot), store)
    assert len(problems) == 1 and 'unknown target' in problems[0] and CELL_POSE in problems[0]

    # the cell's own poses file: the pack's poses plus the cell pose (a whole file, so it holds with whole-file
    # overrides as well as with per-key ones)
    poses = yaml.safe_load((ROBOTS / robot / 'targets.yaml').read_text())
    poses[CELL_POSE] = {'moves': 'hand', 'joints': {'torso': 0.5}, 'elbow': 'up',
                        'left_tool': {'xyz': [0.48, -0.295, 0.6], 'rpy': [0.0, -1.570796, -3.141593]},
                        'right_tool': {'xyz': [0.48, 0.295, 0.6], 'rpy': [0.0, 1.570796, 0.0]}}
    (cell / 'robots' / robot).mkdir(parents=True)
    (cell / 'robots' / robot / 'targets.yaml').write_text(yaml.safe_dump(poses))
    store.reload()
    assert store.robot_file_source(robot, 'targets.yaml')['source'] == 'override'
    assert validate_plan(mission_plan(store, robot, CELL_POSE, tools=True), catalog(store, robot), store) == []
    assert validate_plan(mission_plan(store, robot, 'home'), catalog(store, robot), store) == []


def test_the_carry_envelope_comes_from_the_cell_pack(cell):
    robot = 'galbot_s1'
    navigation = yaml.safe_load((ROBOTS / robot / 'navigation.yaml').read_text())
    (cell / 'robots' / robot).mkdir(parents=True)
    (cell / 'robots' / robot / 'navigation.yaml').write_text(yaml.safe_dump({**navigation, 'carry_envelope': CARRY}))
    store = ConfigStore(cell)
    assert store.navigation(robot)['carry_envelope'] == CARRY
    assert store.navigation(robot)['envelope'] == navigation['envelope']
