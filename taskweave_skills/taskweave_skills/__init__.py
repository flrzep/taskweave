"""Generic, robot-agnostic taskweave skills."""


def shipped_skills() -> list[type]:
    """Every skill class of this package: each ``Skill`` subclass with a ``name`` defined in one of its modules, found
    by scanning the modules (imported, never instantiated). The one list of shipped skills: docs/skills.md and the
    tests of the docs examples take it from here, so a new skill needs no registration there. The classes of the
    six motion skills ``move`` replaced (``parts.PARTS``) are its parts, not skills of a mission: they are left out, and
    so is any skill with ``documented = False``."""
    import importlib
    import pkgutil

    from taskweave_core.skill import Skill

    from .parts import PART_OF_OLD

    found = {}
    for info in pkgutil.iter_modules(__path__):
        module = importlib.import_module(f'{__name__}.{info.name}')
        for value in vars(module).values():
            if (isinstance(value, type) and issubclass(value, Skill) and value.__module__ == module.__name__
                    and not value.__name__.startswith('_') and 'name' in vars(value) and value.name
                    and value.name not in PART_OF_OLD and getattr(value, 'documented', True)):
                found[value.name] = value
    return list(found.values())
