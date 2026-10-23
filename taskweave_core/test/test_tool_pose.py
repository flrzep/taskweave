"""The shared hands fragment of the skill schemas."""

from jsonschema import Draft202012Validator

from taskweave_core.config import schemas
from taskweave_core.validation import oneof_problems


def test_hands_schema_takes_rules_names_and_lists():
    validator = Draft202012Validator(schemas.hands_schema(['left', 'right']))
    assert all(validator.is_valid(v) for v in ('auto', 'any_one', 'all', 'left', ['left', 'right']))
    assert not any(validator.is_valid(v) for v in ('middle', [], ['left', 'left'], ['up']))
    grip = Draft202012Validator(schemas.hands_schema(['left'], ('auto', 'all')))
    assert grip.is_valid('all') and not grip.is_valid('any_one')
    assert oneof_problems(schemas.hands_schema(['left']), allow_several=True) == []
