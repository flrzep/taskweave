"""cli._config_target tells a file in a cell pack from one in a robot pack by the cell's folders."""

from taskweave_core import cli


def pack(folder, *parts):
    folder.mkdir(parents=True)
    (folder / 'pack.yaml').write_text('name: x\nschema_version: 4\n')
    for part in parts:
        (folder / part).mkdir()
    return folder


def test_a_file_in_a_cell_pack_is_a_cell_file(tmp_path):
    cell = pack(tmp_path / 'cell_pack', 'cell', 'objects')
    (cell / 'objects' / 'box.yaml').write_text('name: box\n')
    assert cli._config_target(str(cell / 'objects' / 'box.yaml'), None, 'cell') == (
        str(cell.resolve()), 'objects/box.yaml', 'cell')


def test_a_robot_pack_without_a_profile_is_still_a_robot_pack(tmp_path, capsys):
    robot = pack(tmp_path / 'bot')
    (robot / 'targets.yaml').write_text('{}\n')
    assert cli._config_target(str(robot / 'targets.yaml'), None, 'cell') is None
    assert 'give the cell pack with --cell' in capsys.readouterr().err
    assert cli._config_target(str(robot / 'targets.yaml'), '/some/cell', 'cell') == (
        '/some/cell', 'targets.yaml', 'robot:bot')
