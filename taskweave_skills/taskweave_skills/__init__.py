"""Generic, robot-agnostic taskweave skills."""


def shipped_skills() -> list[type]:
    """Every skill class of this package: each ``Skill`` subclass with a ``name`` defined in one of its modules, found
    by scanning the modules (imported, never instantiated). The one list of shipped skills: docs/skills.md and the
    tests of the docs examples take it from here, so a new skill needs no registration there."""
    import importlib
    import pkgutil

    from taskweave_core.skill import Skill

    found = {}
    for info in pkgutil.iter_modules(__path__):
        module = importlib.import_module(f'{__name__}.{info.name}')
        for value in vars(module).values():
            if (isinstance(value, type) and issubclass(value, Skill) and value.__module__ == module.__name__
                    and not value.__name__.startswith('_') and 'name' in vars(value) and value.name):
                found[value.name] = value
    return list(found.values())
