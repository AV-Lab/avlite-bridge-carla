from avlite.c60_apps.c64_settings_schema import SettingsSchema


class PluginSettingsSchema(SettingsSchema):
    pass


# Settings singleton; filepath is assigned by the plugin loader from the directory name.
PluginSettings = PluginSettingsSchema()
