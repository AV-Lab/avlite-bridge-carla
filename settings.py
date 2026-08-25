from avlite.c60_apps.c64_settings_schema import SettingsSchema


class PluginSettingsSchema(SettingsSchema):
    sync_mode: bool = True
    fixed_delta_seconds: float = 0.05


# Settings singleton; filepath is assigned by the plugin loader from the directory name.
PluginSettings = PluginSettingsSchema()
