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
| `assets/` | Warehouse scene (`clutter/warehouse.usd`), manipulation objects (drill, ball valve), and the operator character (`anim.usd`) for the teleop demo |
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
`--grasp-object <prim>`, `--log-csv auto`, `--arm-gains <preset>`,
`--zed-operator`.

All paths inside the app resolve relative to this folder, so it can be moved
or mounted anywhere as a unit. The operator character (`anim.usd`) references the
standard Isaac Sim People character asset, so the first `--zed-operator` run needs
the Isaac Sim asset root to be reachable (cloud or a local mirror), like any other
Isaac Sim built-in asset.

## Teleoperation demo (operator mirroring)

This is the setup behind the demo GIF in the top-level README: a virtual operator
in the scene is filmed by a virtual ZED camera, the operator-facing perception
stack tracks the wrist, and the robot arm mirrors it, all in simulation.

It needs one external dependency, the **Stereolabs ZED Isaac Sim extension**
(`sl.sensor.camera`), which provides the virtual ZED streamer and the `ZED_X`
camera asset. It is not vendored here (third-party, large). Clone and build it
from [`stereolabs/zed-isaac-sim`](https://github.com/stereolabs/zed-isaac-sim),
then point the app at its `exts/` folder:

```bash
export ZED_ISAAC_EXTS=/path/to/zed-isaac-sim/exts   # default: /workspace/zed-isaac-sim/exts
```

With that in place, bring up the full mirror loop (all ROS 2 containers share
`ROS_DOMAIN_ID=8`):

```bash
# 1. isaac-sim container: spawn the operator + virtual ZED and start streaming.
#    --arm-gains sh10_el0-8_w5 is the stiffer preset used for the demo.
/workspace/spot_warehouse/applications/run_spot_warehouse.sh \
    --zed-operator --arm-gains sh10_el0-8_w5
#    Press Play so the streaming graph starts ticking.

# 2. zed container: receive the sim stream as a virtual ZED X.
ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:=zedx sim_mode:=true use_sim_time:=true

# 3. spot-ros2 container: track the operator's wrist and drive the arm.
ros2 run arm_pose_estimator wrist_detector --ros-args \
    -p color_topic:=/zed/zed_node/rgb/image_rect_color \
    -p depth_topic:=/zed/zed_node/depth/depth_registered \
    -p camera_info_topic:=/zed/zed_node/depth/camera_info \
    -p filter_alpha_wrist:=0.5
ros2 launch spot_operation_ros2 curobo_mpc.launch.py use_ros2_control:=false
```

Then, in the Isaac Sim viewport, **drag `/World/Operator/wrist_target`**: the
operator's arm follows it via an inverse-kinematics graph, the wrist detector
picks up the motion from the ZED stream, and the robot arm mirrors it. Aim the
operator and camera by moving `/World/Operator` and `/World/ZED_X` in the GUI,
then press **`G`** to persist that framing (and the robot's pose) to
`zed_operator_poses.json` so the next `--zed-operator` run restores it.

Note: driving the operator hands-free from a script is not currently supported;
the arm follows only a live drag of `wrist_target` in the GUI.

## Attribution

`spot_warehouse.py` and `spot_policy.py` are modified versions of the
Apache-2.0 licensed IsaacRobotics project; `params/env.yaml` and
`policies/spot_arm_policy.pt` are unmodified upstream files. See `NOTICE` and
`LICENSE`.
