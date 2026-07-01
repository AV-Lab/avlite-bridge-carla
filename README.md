# avlite-bridge-carla

CARLA simulator world bridge for AVLite. Registers `CarlaBridge` — connects to a running CARLA server and exposes ego state, sensors, and control.

**Plugin name:** `avlite-bridge-carla`

## Install

```bash
git clone https://github.com/AV-Lab/avlite-bridge-carla \
  ~/.local/share/avlite/plugins/avlite-bridge-carla
```

Requires [AVLite](https://github.com/AV-Lab/avlite) and a running [CARLA](https://github.com/carla-simulator/carla) instance.

Monorepo path: `related-repos/avlite-bridge-carla`

## Configuration

```yaml
c40_community_plugins:
  avlite-bridge-carla: avlite-bridge-carla   # or related-repos/avlite-bridge-carla
c40_bridge: CarlaBridge
```

Plugin settings: `~/.config/avlite/plugin_avlite-bridge-carla.yaml` (defaults in `config/default.yaml`).

## Requirements

Install CARLA Python API for your CARLA version, then:

```bash
pip install -r requirements.txt
```

Start CARLA before launching AVLite with a profile that selects `CarlaBridge`.
