from pathlib import Path

import yaml

from avlite.c10_perception.c19_settings import PerceptionSettingsSchema
from avlite.c40_execution.c49_settings import ExecutionSettingsSchema
from avlite.c60_apps.c69_settings import AppSettingsSchema

from settings import PluginSettingsSchema


PROFILE_NAME = "avlite-bridge-carla"
PROFILE_PATH = Path(__file__).parents[1] / f"{PROFILE_NAME}.yaml"


def _strings(value):
    if isinstance(value, dict):
        for key, child in value.items():
            yield str(key)
            yield from _strings(child)
    elif isinstance(value, list):
        for child in value:
            yield from _strings(child)
    elif isinstance(value, str):
        yield value


def test_recommended_profile_is_portable_and_valid():
    profile = yaml.safe_load(PROFILE_PATH.read_text())

    PerceptionSettingsSchema.model_validate(profile["c10_perception"])
    ExecutionSettingsSchema.model_validate(profile["c40_execution"])
    AppSettingsSchema.model_validate(profile["c69_apps"])
    PluginSettingsSchema.model_validate(profile["plugins"][PROFILE_NAME])

    assert profile["c69_apps"]["c62_community_plugins"] == {
        PROFILE_NAME: PROFILE_NAME,
    }
    assert set(profile["plugins"]) == {PROFILE_NAME}
    assert not any(Path(value).is_absolute() for value in _strings(profile))


def test_recommended_profile_uses_carla_ground_truth():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    perception = profile["c10_perception"]
    execution = profile["c40_execution"]

    assert perception["c12_detection_strategy"] == ""
    assert perception["c12_tracking_strategy"] == ""
    assert execution["c40_localization"] == ""
    assert set(execution["c41_world_stack_capabilities"]) == {
        "DETECTION",
        "TRACKING",
        "LOCALIZATION",
    }
