import importlib


def test_package_imports():
    assert importlib.import_module('taskweave_skills') is not None
