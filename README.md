# From Perception to Assistance: Open-Vocabulary Shared Autonomy for Robotic Manipulation

Anonymized code release accompanying the RA-L submission. The stack implements
vision-based shared-control teleoperation of a quadruped mobile manipulator
(Boston Dynamics Spot with the 6-DoF Spot Arm): a calibration-free camera
interface decodes operator intent, an open-vocabulary perception pipeline
grounds a free-form text prompt into a 3D grasp frame, and a GPU-accelerated
model-predictive controller tracks the (potential-field assisted) reference
under self- and environment-collision constraints built from onboard volumetric
mapping.

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

`fake_wrist_target.py` publishes a synthetic operator reference for bench tests
without the camera interface. `isaac_publisher.py`, `joint_state_mapper.py`,
and `joint_state_remapper.py` bridge joint topics for simulation runs.

## Running

The stack is containerized; services are defined in `docker-compose.yaml`:

- `spot-ros2` — ROS 2 Humble workspace (perception, control, teleop interface)
- `zed` — ZED 2i camera driver for the operator-facing camera
- `isaac-ros` — nvblox volumetric mapping
- `vllm-server` — Qwen3-VL-4B-Instruct served over an OpenAI-compatible API

Typical bring-up on the robot:

```bash
docker compose up -d vllm-server zed spot-ros2 isaac-ros
# inside spot-ros2 (after colcon build --symlink-install):
ros2 launch arm_pose_estimator wrist_detector_zed.launch.py       # operator interface
ros2 launch spot_operation_ros2 perception_minimal.launch.py \
     object_prompt:="wheel valve"                                  # grounding + tracking
ros2 launch spot_operation_ros2 curobo_mpc.launch.py               # collision-aware MPC
# inside isaac-ros:
ros2 launch spot_nvblox spot_nvblox.launch.py                      # TSDF/ESDF mapping
```

The MPC node is executed with a dedicated virtual environment
(`spot-ros2_ws/curobo_venv`, referenced by `curobo_mpc.launch.py`) in which
cuRobo and its PyTorch dependencies are installed inside the `spot-ros2`
container, following the upstream cuRobo installation instructions.

## Dependencies (pinned)

| Dependency | Version / pin | Notes |
|---|---|---|
| [cuRobo](https://github.com/NVlabs/curobo) | v0.7.7 | MPPI MPC, ESDF collision checking |
| [isaac_ros_nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox) | 7908a18 (v3.2 line) | TSDF/ESDF mapping |
| [isaac_ros_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common) | fcf4d9e (v3.2 line) | build/runtime support |
| [spot_ros2](https://github.com/bdaiinstitute/spot_ros2) | spot-sdk-4.0.0 base | robot driver; used with minor launch adjustments |
| [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) | e9f5490 (humble-v4.2.5 line) | operator camera |
| Qwen3-VL-4B-Instruct | via vLLM (see `docker-compose.yaml`) | open-vocabulary grounding |
| SAM 2.1 (base) | via `ultralytics` | promptable video segmentation |
| MediaPipe Pose / Hands / Gestures | `mediapipe` | operator tracking |

`setup_dependencies.sh` clones the dependencies above into the expected
workspace paths.

## License

Code is provided for peer review. A license will be added upon acceptance.
