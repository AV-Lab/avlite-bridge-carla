from avlite.c60_apps.c64_settings_schema import SettingsSchema
from pydantic import Field


class PluginSettingsSchema(SettingsSchema):
    sync_mode: bool = True
    sensor_timeout: float = Field(default=2.0, gt=0)
    sensor_queue_size: int = Field(default=64, ge=2)
    max_sensor_age: float = Field(default=0.5, gt=0)
    seed: int = Field(default=0, ge=0)


# Settings singleton; filepath is assigned by the plugin loader from the directory name.
PluginSettings = PluginSettingsSchema()
