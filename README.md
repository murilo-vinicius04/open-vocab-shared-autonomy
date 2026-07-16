# From Perception to Assistance: Open-Vocabulary Shared Autonomy for Robotic Manipulation

Code release for the paper *"From Perception to Assistance: Open-Vocabulary Shared Autonomy for Robotic Manipulation"*.

The stack implements vision-based shared-control teleoperation of a quadruped mobile manipulator (Boston Dynamics Spot with the 6-DoF Spot Arm). A calibration-free camera interface decodes operator intent, an open-vocabulary perception pipeline grounds a free-form text prompt into a 3D grasp frame, and a GPU-accelerated model-predictive controller tracks the (potential-field assisted) reference under self- and environment-collision constraints built from onboard volumetric mapping. An autonomous mode can be gesture-triggered to complete the grasp on the same grounded target.

## Video

[Watch the demonstration video](https://drive.google.com/file/d/1C962rltk_xPM-4pfOvlf88CGkEJcCKJg/view)

The full demonstration includes the industrial valve manipulation and
pick-and-place tasks, the collision avoidance stress test, and autonomous
execution.

## Framework overview

![Framework overview](docs/framework_overview.png)

The operator is tracked with a ZED 2i RGB-D camera and MediaPipe, with no wearables, fiducials, or calibration stage. Wrist motion maps to an end-effector position reference, the palm normal sets the gripper roll, and hand gestures command the gripper and mode switches. The target is specified with a free-form text prompt ("wheel valve"), grounded by Qwen3-VL in the gripper camera, and tracked across the three onboard cameras with SAM 2 streaming predictors, producing a world-latched grasp frame that is kept out of the static obstacle map. nvblox fuses the onboard stereo depth into a TSDF/ESDF, and cuRobo runs MPPI-based MPC at 50 Hz against that map for self- and environment-collision avoidance. During the final approach, an attractive potential field corrects the operator's reference toward the grasp frame while the operator retains authority. The same grasp frame drives the gesture-triggered autonomous mode.

## Repository layout and paper-section map

| Paper section | Component | Location |
|---|---|---|
| II-A Vision-based teleoperation | Body/wrist tracking, body frame estimation, workspace scaling | `spot-ros2_ws/src/arm_pose_estimator/arm_pose_estimator/wrist_detector.py` |
| II-A Hand pose and gestures | Palm-normal roll, gesture commands (MediaPipe) | `arm_pose_estimator/hand_pose_estimator.py`, `hand_orientation_estimator.py` |
| II-B Volumetric mapping | nvblox TSDF/ESDF configuration and launch, dynamic-mask integration | `isaac-ros_ws/src/spot_nvblox/` |
| II-C Open-vocabulary grounding | VLM grounding service (Qwen3-VL via vLLM), affordance-point selection | `spot_operation_ros2/vlm_relocalize_node.py` |
| II-C Multi-camera segmentation | SAM 2 streaming predictors (hand + two body cameras), seeding, lifecycle | `spot_operation_ros2/sam2_tracker_node.py`, `tf_projection_node.py`, `coordinator_node.py`, `image_roll.py` |
| II-D Collision-aware MPC | cuRobo MPPI MPC node, ESDF interface, collision spheres | `spot_operation_ros2/curobo_mpc_node.py`, `config/` |
| II-E Potential-field assistance | Attractive field toward the grasp frame (inside the MPC node goal update) | `spot_operation_ros2/curobo_mpc_node.py` |
| II-F Autonomous execution | Gesture-triggered mode switch; the MPC tracks the grasp frame directly; gripper control | `spot_operation_ros2/curobo_mpc_node.py`, `control_mode_switcher.py`, `gripper_controller.py` |
| Simulation | Isaac Sim warehouse demo: Spot + arm driven by the locomanipulation policy (TorchScript), ROS 2 bridged | `isaac-sim_ws/spot_warehouse/` |

`fake_wrist_target.py` publishes a synthetic operator reference for bench tests without the camera interface.

## Hardware and compute

The experiments in the paper use:

- Boston Dynamics Spot with the 6-DoF Spot Arm and gripper camera
- ZED 2i RGB-D camera facing the operator
- A single NVIDIA GPU running the full onboard-facing stack (nvblox mapping, cuRobo MPC, SAM 2 trackers) plus the vLLM server for Qwen3-VL-4B-Instruct

The teleoperation interface itself is robot-agnostic: it publishes a Cartesian end-effector reference, a roll command, and discrete gripper actions, and can be adapted to any RGB-D sensor with aligned depth and known intrinsics.

## Prerequisites

The stack runs in GPU containers, so the host needs:

- An NVIDIA GPU with a recent driver and the
  [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
  (every compute service uses `runtime: nvidia` / `gpus: all`).
- Docker with the Compose v2 plugin (`docker compose`).
- An X server for the GUI tools (RViz, Isaac Sim). Allow local containers to
  connect and export the display, e.g. `xhost +local:` and `export DISPLAY=:0`.

The base images (Isaac Sim, Isaac ROS, ZED, the Qwen vLLM image) are pulled from
public registries on first build and total tens of GB; no NGC login is required.
Environment variables read by `docker-compose.yaml`:

- `HUGGING_FACE_HUB_TOKEN` - only needed for gated Hugging Face models.
  Qwen3-VL-4B-Instruct is ungated, so this can be left unset (compose prints a
  harmless "variable is not set" warning).
- `SPOT_NAME` - name of the Spot robot (defaults to `Spot`).
- `DISPLAY` - forwarded to the GUI containers.

## Building

Third-party ROS dependencies are git submodules. Clone into a directory named
`spot-teleop` (this is the conventional name; `docker-compose.yaml` bind-mounts
the checkout to `/home/spot-teleop` inside the containers). Replace
`<repository-url>` with this repository's clone URL:

```bash
# Clone with all submodules.
git clone --recursive <repository-url> spot-teleop && cd spot-teleop

# Or, in an existing checkout:
git submodule update --init --recursive
```

`git submodule update --init --recursive` checks out each submodule at the exact
commit recorded here, so the version-critical dependencies (cuRobo, nvblox,
isaac_ros_common, ZED) always land on the commit used in the experiments. The
Spot driver stack (`spot_ros2`) tracks upstream `main` and pulls its own
sub-packages (`spot_wrapper`, `spot_description`, `synchros2`) through the same
recursive update; run `git submodule update --remote spot-ros2_ws/src/spot_ros2`
to advance it. Do not use a bare `--remote` on the whole tree, as that would
move the pinned dependencies off their recorded commits.

### Building the ROS 2 workspace

The containers bind-mount the repository over `/home/spot-teleop`, which shadows
the source that was copied in at image-build time. The workspace is therefore
prepared at runtime, inside the container, against the mounted tree:

```bash
docker compose up -d spot-ros2
docker compose exec spot-ros2 bash

# --- inside the spot-ros2 container ---
cd /home/spot-teleop/spot-ros2_ws
rosdep update && rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# Run the Spot driver setup against the mounted tree. It installs the Spot SDK
# dependencies and re-pins setuptools to the version ROS 2 Humble expects (the
# image ships a newer setuptools that otherwise breaks ament_python builds):
cd src/spot_ros2 && ./install_spot_ros2.sh && cd ../..

colcon build --symlink-install
source install/setup.bash
```

The entrypoint auto-sources `install/setup.bash` only if it already exists, so on
the first run you must build and then `source` it manually in that shell; later
`docker compose exec` sessions and container restarts source it for you. The
`zed_ws` and `isaac-ros_ws` workspaces are built the same way inside their own
containers.

### cuRobo MPC environment

`curobo_mpc.launch.py` runs the MPC node from a dedicated virtual environment
(`spot-ros2_ws/curobo_venv`) so cuRobo and its CUDA-compiled kernels stay
isolated from the ROS Python environment. The `spot-ros2` image provides the
CUDA runtime through the NVIDIA container runtime, but not the CUDA compiler or
the `venv` tooling, so install those first. Inside the `spot-ros2` container:

```bash
# 1. Build prerequisites the base image does not include. The CUDA compiler
#    comes from NVIDIA's apt repo (not the default sources); python3.10-venv is
#    in the Ubuntu archive.
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
dpkg -i cuda-keyring_1.1-1_all.deb
apt-get update && apt-get install -y cuda-toolkit-12-8 python3.10-venv
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=$CUDA_HOME/bin:$PATH

# 2. Create the venv and install a CUDA-matched PyTorch.
cd /home/spot-teleop/spot-ros2_ws
python3 -m venv curobo_venv
source curobo_venv/bin/activate
pip install --upgrade pip wheel "setuptools<70" setuptools_scm
pip install torch --index-url https://download.pytorch.org/whl/cu128

# 3. Build cuRobo (compiles CUDA kernels, ~15 min), then pin warp-lang.
#    cuRobo 0.7.7 uses the older warp API, so pin it explicitly (the latest
#    warp-lang removes the wp.torch attribute cuRobo relies on).
pip install -e src/curobo --no-build-isolation
pip install "warp-lang==1.3.0"
deactivate
```

The MPC node imports message types from `nvblox_msgs` (part of the
`isaac_ros_nvblox` submodule) even when the ESDF is disabled, so build that one
package into the workspace and re-source (run from `spot-ros2_ws`):

```bash
colcon build --paths ../isaac-ros_ws/src/isaac_ros_nvblox/nvblox_msgs \
             --packages-select nvblox_msgs
source install/setup.bash
```

## Running

The stack is containerized; services are defined in `docker-compose.yaml`:

- `spot-ros2` — ROS 2 Humble workspace (perception, control, teleop interface)
- `zed` — ZED 2i camera driver for the operator-facing camera
- `isaac-ros` — nvblox volumetric mapping
- `vllm-server` — Qwen3-VL-4B-Instruct served over an OpenAI-compatible API
- `isaac-sim` — NVIDIA Isaac Sim, for the simulated warehouse demo (optional)

Typical bring-up on the robot:

```bash
docker compose up -d vllm-server zed spot-ros2 isaac-ros
# inside spot-ros2 (after building the workspace, see "Building the ROS 2 workspace"):
ros2 launch arm_pose_estimator wrist_detector_zed.launch.py       # operator interface
ros2 launch spot_operation_ros2 perception_minimal.launch.py \
     object_prompt:="wheel valve"                                  # grounding + tracking
ros2 launch spot_operation_ros2 curobo_mpc.launch.py               # collision-aware MPC
# inside isaac-ros:
ros2 launch spot_nvblox spot_nvblox.launch.py                      # TSDF/ESDF mapping
```

The `curobo_mpc.launch.py` node runs from the `curobo_venv` virtual environment
described under "Building the ROS 2 workspace" above.

`spot_nvblox` defaults to `use_segmentation:=false`. Only set it to `true` once
`perception_minimal.launch.py` is running, since that is what publishes the
SAM 2 masks nvblox's synchronizer waits on; enabling segmentation without those
masks stalls the mapper. The `isaac-ros_ws` workspace is built the same way as
`spot-ros2_ws` (`rosdep install` pulls the Isaac ROS NITROS/GXF dependencies);
give it a few GB of free disk, as that dependency set is large.

## Isaac Sim locomanipulation demo

`isaac-sim_ws/spot_warehouse/` is a self-contained Isaac Sim application that
spawns Spot with the arm in a cluttered warehouse and drives it with the
locomanipulation policy (a self-contained TorchScript module shipped at
`spot_warehouse/policies/spot_warehouse_policy.pt`). It publishes the same
ROS 2 topics and joint names as the real robot (`/joint_states`, `/tf`,
`/arm/joint_command`), so the perception and MPC stacks above run against the
simulation unchanged, with no sim-specific launch flags.

```bash
docker compose up -d isaac-sim
docker compose exec isaac-sim bash
# inside the container (GUI mode; the workspace is mounted at /workspace):
/workspace/spot_warehouse/applications/run_spot_warehouse.sh
# optionally: --policy <path/to/policy.pt>, --obs-mode {loco,arm}
```

See `isaac-sim_ws/spot_warehouse/README.md` for details and attribution
(the app derives from the Apache-2.0 IsaacRobotics project).

## Dependencies (pinned)

The ROS dependencies below are git submodules (see `.gitmodules`). The
version-critical ones are pinned to an exact commit; the Spot driver stack
tracks upstream `main`.

| Dependency | Pinned version | Notes |
|---|---|---|
| [cuRobo](https://github.com/NVlabs/curobo) | `ebb7170` (v0.7.7+5) | MPPI MPC, ESDF collision checking |
| [isaac_ros_nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox) | `7908a18` (v3.2 line) | TSDF/ESDF mapping |
| [isaac_ros_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common) | `fcf4d9e` (v3.2 line) | build/runtime support |
| [spot_ros2](https://github.com/rai-opensource/spot_ros2) | `main` | robot driver (pulls `spot_wrapper`, `spot_description`, `synchros2`) |
| [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) | `e9f5490` (humble-v4.2.5 line) | operator camera |
| [zed-ros2-interfaces](https://github.com/stereolabs/zed-ros2-interfaces) | `cfffb88` (5.0.1+) | ZED message definitions |
| Qwen3-VL-4B-Instruct | via vLLM (see `docker-compose.yaml`) | open-vocabulary grounding |
| SAM 2.1 (tiny) | via `ultralytics` | promptable video segmentation |
| MediaPipe Pose / Hands / Gestures | `mediapipe` | operator tracking |

These models are not vendored here. The `ultralytics` and `mediapipe` packages
are installed into the images at build time, but their weights are not: the SAM 2
checkpoint (`sam2.1_t.pt`, about 75 MB) is fetched by `ultralytics` the first time
the tracker runs, and the Qwen3-VL weights are cached by the vLLM server on first
launch into the mounted Hugging Face cache. The first run of each therefore needs
network access.

