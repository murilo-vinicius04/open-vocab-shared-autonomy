# Spot warehouse locomanipulation demo (Isaac Sim)

Standalone Isaac Sim application that spawns Spot with the 6-DoF arm in a
cluttered warehouse and drives it with the locomanipulation policy used in the
paper's simulation experiments. The robot is imported from the URDF in
`robot/spot/`, the policy is a self-contained TorchScript module, and the app
exposes the same ROS 2 topics as the real-robot stack (joint states, cameras,
arm commands), so the perception and control workspaces in this repository can
run against it.

## Layout

| Path | Content |
|---|---|
| `applications/` | The warehouse app, launcher, camera/ROS-bridge/arm-command helpers, phase-2 config |
| `policies/spot_warehouse_policy.pt` | Locomanipulation policy (TorchScript, obs 69 → act 19), the default in `loco` mode |
| `policies/spot_arm_policy.pt` | Upstream flat-terrain arm policy, used with `--obs-mode arm` |
| `params/env.yaml` | Joint properties/defaults consumed by the policy loader |
| `assets/` | Warehouse scene (`clutter/warehouse.usd`) and manipulation objects (drill, ball valve) |
| `robot/spot/` | `spot_with_arm.urdf`, meshes, and the arm reachability table |

## Running

Requires NVIDIA Isaac Sim (tested with the `isaac-sim` service in the
repository's `docker-compose.yaml`, which mounts `isaac-sim_ws/` at
`/workspace`):

```bash
docker compose up -d isaac-sim
docker compose exec isaac-sim bash
# inside the container:
/workspace/spot_warehouse/applications/run_spot_warehouse.sh
# a different policy checkpoint can be supplied explicitly:
/workspace/spot_warehouse/applications/run_spot_warehouse.sh --policy <path/to/policy.pt>
```

Useful flags (see `applications/spot_warehouse.py`): `--obs-mode {loco,arm}`,
`--grasp-object <prim>`, `--log-csv auto`.

All paths inside the app resolve relative to this folder, so it can be moved
or mounted anywhere as a unit.

## Attribution

`spot_warehouse.py` and `spot_policy.py` are modified versions of the
Apache-2.0 licensed IsaacRobotics project; `params/env.yaml` and
`policies/spot_arm_policy.pt` are unmodified upstream files. See `NOTICE` and
`LICENSE`.
