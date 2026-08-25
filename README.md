# avlite-bridge-carla

CARLA simulator world bridge for AVLite. Registers `Carla4Bridge` — connects to a running CARLA server and exposes ego state, sensors, and control.

**Plugin name:** `avlite-bridge-carla`

## Install

```bash
git clone https://github.com/AV-Lab/avlite-bridge-carla \
  ~/.local/share/avlite/plugins/avlite-bridge-carla
```

Requires [AVLite](https://github.com/AV-Lab/avlite) and a running [CARLA](https://github.com/carla-simulator/carla) instance.

Monorepo path: `related-repos/avlite-bridge-carla`

## Configuration

Register the plugin and select the bridge in your profile YAML (e.g.
`~/.config/avlite/<profile>.yaml`):

```yaml
c69_apps:
  c62_community_plugins:
    avlite-bridge-carla: avlite-bridge-carla   # or a repo-relative/absolute path
c40_execution:
  c40_bridge: Carla4Bridge
```

Plugin settings: `~/.config/avlite/plugin_avlite-bridge-carla.yaml`. Shipped defaults (monorepo / AVLite clone): `configs/plugin_avlite-bridge-carla.yaml`.

See [SYNC_ASYNC.md](SYNC_ASYNC.md) for `sync_mode`/`fixed_delta_seconds` —
whether CARLA free-runs in real time or only advances a fixed step per tick,
and how that affects LiDAR/camera synchronization.

## Requirements

Install CARLA Python API for your CARLA version, then:

```bash
pip install -r requirements.txt
```

Start CARLA before launching AVLite with a profile that selects `Carla4Bridge`.
