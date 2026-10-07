"""Public exports for the AVLite CARLA world-bridge plugin."""

# AVLite loads this file as ``avlite.plugins.avlite_bridge_carla``. Pytest also
# collects it directly as ``__init__`` from this dashed checkout directory.
if __package__:
    from .carla_bridge import CarlaBridge, CarlaSensorFrame
    from .settings import PluginSettings, PluginSettingsSchema

    __all__ = [
        "CarlaBridge",
        "CarlaSensorFrame",
        "PluginSettings",
        "PluginSettingsSchema",
    ]
else:
    __all__ = []
