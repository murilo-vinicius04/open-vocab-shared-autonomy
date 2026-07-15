from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import csv
import logging
import numpy as np
import os
import signal
import sys
import termios
import threading
import time
import tty
from pathlib import Path

from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from omni.isaac.core.utils.extensions import enable_extension

# Custom warehouse (simple_warehouse + table + clutter objects pre-arranged)
WAREHOUSE_USD = str(Path(__file__).resolve().parent.parent / "assets" / "clutter" / "warehouse.usd")

from spot_cameras import ALL_CAMERAS, create_spot_cameras, initialize_cameras
from spot_ros_bridge import ROSBridgeBuilder
from spot_arm_subscriber import SpotArmCommandSubscriber
from spot_policy import (
    SpotLocoPolicy,
    apply_trained_gains,
    load_csv_poses,
    load_spot_loco_phase2_config,
)

# Configure logging
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)

enable_extension("isaacsim.ros2.bridge")
try:
    enable_extension("isaacsim.asset.importer.urdf")
except Exception:
    try:
        enable_extension("omni.importer.urdf")
    except Exception:
        enable_extension("omni.isaac.urdf")

try:
    from isaacsim.asset.importer.urdf import _urdf
    _URDF_IMPORTER_FLAVOR = "isaacsim"
except ImportError:
    try:
        from omni.importer.urdf import _urdf
        _URDF_IMPORTER_FLAVOR = "omni.importer"
    except ImportError:
        from omni.isaac.urdf import _urdf
        _URDF_IMPORTER_FLAVOR = "omni.isaac"

if _URDF_IMPORTER_FLAVOR == "isaacsim":
    import omni.kit.commands


ARM_JOINTS = ["arm_sh0", "arm_sh1", "arm_el0", "arm_el1", "arm_wr0", "arm_wr1", "arm_f1x"]


def _arm_gains(sh: float, el0_kd: float, el1_wr: float) -> dict:
    """Build an arm gain table parameterised by shoulder, el0, and el1/wrist kd."""
    return {
        "arm_sh0": (120.0, sh),
        "arm_sh1": (120.0, sh),
        "arm_el0": (120.0, el0_kd),
        "arm_el1": (100.0, el1_wr),
        "arm_wr0": (100.0, el1_wr),
        "arm_wr1": (100.0, el1_wr),
        "arm_f1x": (16.0, 0.32),
    }


# Sweep presets (rotated each interval). Each entry is (name, gains_dict).
# Format key: sh<kd_sh>_el0<kd_el0>_w<kd_el1_wr>
ARM_GAIN_SWEEP = [
    ("sh2_el0-2_w2",   _arm_gains(2.0, 2.0, 2.0)),   # original baseline
    ("sh4_el0-3_w3",   _arm_gains(4.0, 3.0, 3.0)),
    ("sh6_el0-5_w3",   _arm_gains(6.0, 5.0, 3.0)),   # prev recommendation (with wrist bump)
    ("sh8_el0-6_w4",   _arm_gains(8.0, 6.0, 4.0)),   # current NEW
    ("sh10_el0-8_w5",  _arm_gains(10.0, 8.0, 5.0)),  # near/over critical
]


def apply_arm_gains(robot, gains: dict) -> None:
    """Set kp/kd on the specified arm joints of a SingleArticulation."""
    indices, kps, kds = [], [], []
    for jname, (kp, kd) in gains.items():
        try:
            idx = robot.get_dof_index(jname)
        except Exception:
            continue
        indices.append(idx)
        kps.append(kp)
        kds.append(kd)
    if not indices:
        return
    robot._articulation_view.set_gains(
        kps=np.asarray(kps, dtype=np.float32),
        kds=np.asarray(kds, dtype=np.float32),
        joint_indices=indices,
    )


class ArmDataLogger:
    """Per-physics-step CSV logger for arm joint state, target, and active gain set."""

    def __init__(self, path: str, joint_names: list[str]) -> None:
        self._joint_names = list(joint_names)
        self._path = path
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._fh = open(path, "w", newline="")
        self._writer = csv.writer(self._fh)
        header = [
            "t_sim", "gain_mode", "tgt_source",
            "pose_idx", "difficulty",
            "cmd_vx", "cmd_vy", "cmd_wz",
            "base_lin_vel_x", "base_lin_vel_y", "base_lin_vel_z",
            "base_ang_vel_x", "base_ang_vel_y", "base_ang_vel_z",
            "proj_grav_x", "proj_grav_y", "proj_grav_z",
            # grasped-object world pose+vel and end-effector world pose, so the
            # lift can be judged on the TASK (did the bottle rise / slip?) not
            # just joint torques. obj_* from the bottle rigid body, ee_* from
            # arm_link_wr1 (hand/palm). NaN when the handles aren't available.
            "obj_x", "obj_y", "obj_z", "obj_vx", "obj_vy", "obj_vz",
            "ee_x", "ee_y", "ee_z",
        ]
        for j in self._joint_names:
            header += [f"{j}_tgt", f"{j}_pos", f"{j}_vel", f"{j}_eff"]
        self._writer.writerow(header)
        self._t = 0.0
        self._joint_indices = None  # resolved lazily once robot.dof_names is ready
        self._policy_idx = None     # index into policy arm order

    def resolve_indices(self, robot, policy_arm_order: list[str], extra_meta: dict | None = None) -> None:
        self._joint_indices = [robot.get_dof_index(j) for j in self._joint_names]
        self._policy_idx = [policy_arm_order.index(j) for j in self._joint_names]
        # Capture the per-joint effort (torque) limits once and drop them in a
        # sidecar JSON so the plot script can draw saturation lines. If the arm
        # can't lift the object, the measured effort will be pinned at this limit.
        try:
            max_eff = np.asarray(robot._articulation_view.get_max_efforts()).reshape(-1)
            limits = {j: float(max_eff[i]) for j, i in zip(self._joint_names, self._joint_indices)}
        except Exception as exc:  # pragma: no cover - depends on Isaac build
            print(f"[ArmLogger] could not read effort limits: {exc}")
            limits = {j: float("nan") for j in self._joint_names}
        try:
            import json
            meta = {"effort_limits": limits, "joint_names": self._joint_names}
            if extra_meta:
                meta.update(extra_meta)
            with open(self._path + ".meta.json", "w") as fh:
                json.dump(meta, fh, indent=2)
            print(f"[ArmLogger] effort limits: {limits}")
            if extra_meta:
                print(f"[ArmLogger] extra meta: {extra_meta}")
        except Exception:
            pass

    def log(
        self,
        dt: float,
        gain_mode: str,
        robot,
        target_array,
        tgt_source: str,
        pose_idx: int,
        difficulty: str,
        cmd: np.ndarray,
        base_lin_vel_b: np.ndarray,
        base_ang_vel_b: np.ndarray,
        projected_gravity_b: np.ndarray,
        obj_pos: np.ndarray | None = None,
        obj_vel: np.ndarray | None = None,
        ee_pos: np.ndarray | None = None,
    ) -> None:
        """target_array is 7-DOF in policy arm order (sh0,sh1,el0,el1,wr0,wr1,f1x)."""
        self._t += dt
        pos = np.asarray(robot.get_joint_positions(), dtype=np.float32)
        vel = np.asarray(robot.get_joint_velocities(), dtype=np.float32)
        try:
            eff = np.asarray(robot.get_measured_joint_efforts(), dtype=np.float32).reshape(-1)
        except Exception:
            eff = np.full(pos.shape, np.nan, dtype=np.float32)
        row = [
            f"{self._t:.4f}", gain_mode, tgt_source,
            int(pose_idx), difficulty,
            f"{float(cmd[0]):.4f}", f"{float(cmd[1]):.4f}", f"{float(cmd[2]):.4f}",
            f"{float(base_lin_vel_b[0]):.5f}", f"{float(base_lin_vel_b[1]):.5f}", f"{float(base_lin_vel_b[2]):.5f}",
            f"{float(base_ang_vel_b[0]):.5f}", f"{float(base_ang_vel_b[1]):.5f}", f"{float(base_ang_vel_b[2]):.5f}",
            f"{float(projected_gravity_b[0]):.5f}", f"{float(projected_gravity_b[1]):.5f}", f"{float(projected_gravity_b[2]):.5f}",
        ]
        op = np.asarray(obj_pos).reshape(-1) if obj_pos is not None else np.full(3, np.nan)
        ov = np.asarray(obj_vel).reshape(-1) if obj_vel is not None else np.full(3, np.nan)
        ep = np.asarray(ee_pos).reshape(-1) if ee_pos is not None else np.full(3, np.nan)
        row += [
            f"{op[0]:.5f}", f"{op[1]:.5f}", f"{op[2]:.5f}",
            f"{ov[0]:.5f}", f"{ov[1]:.5f}", f"{ov[2]:.5f}",
            f"{ep[0]:.5f}", f"{ep[1]:.5f}", f"{ep[2]:.5f}",
        ]
        for ji, pi in zip(self._joint_indices, self._policy_idx):
            tgt = float(target_array[pi]) if target_array is not None else float("nan")
            row += [f"{tgt:.5f}", f"{pos[ji]:.5f}", f"{vel[ji]:.5f}", f"{eff[ji]:.5f}"]
        self._writer.writerow(row)
        self._fh.flush()  # stream to disk each step so a sim crash mid-lift loses nothing

    def close(self) -> None:
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:
            pass


def _import_spot_from_urdf(urdf_path: str, prim_path: str) -> str:
    """Import the Spot URDF using the Isaac Sim URDF importer."""
    logger.info("Importing Spot URDF: %s", urdf_path)
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.fix_base = False
    import_config.make_default_prim = True
    import_config.self_collision = False
    import_config.create_physics_scene = False
    import_config.import_inertia_tensor = True
    import_config.default_drive_strength = 60.0
    import_config.default_position_drive_damping = 1.5
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.distance_scale = 1
    import_config.density = 0.0

    if _URDF_IMPORTER_FLAVOR == "isaacsim":
        dest_path = str(Path("/tmp") / f"{Path(urdf_path).stem}_imported.usd")
        result, _ = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=urdf_path,
            import_config=import_config,
            dest_path=dest_path,
        )
        if not result:
            raise RuntimeError(f"URDFParseAndImportFile failed for {urdf_path}")
        add_reference_to_stage(dest_path, prim_path)
        return prim_path

    urdf_dir = os.path.dirname(urdf_path)
    urdf_file = os.path.basename(urdf_path)
    urdf_interface = _urdf.acquire_urdf_interface()
    imported_robot = urdf_interface.parse_urdf(urdf_dir, urdf_file, import_config)
    return urdf_interface.import_robot(urdf_dir, urdf_file, imported_robot, import_config, prim_path)


def _start_keyboard_listener(callbacks: dict) -> None:
    """Read single keypresses from stdin (raw mode) in a daemon thread."""

    def _loop():
        fd = sys.stdin.fileno()
        try:
            old_attrs = termios.tcgetattr(fd)
        except termios.error:
            # No TTY attached (e.g. detached container): keyboard control off.
            carb.log_warn("stdin is not a TTY; keyboard teleop disabled")
            return
        try:
            tty.setraw(fd)
            while True:
                ch = sys.stdin.buffer.read(1)
                if ch == b"\x03":
                    os.kill(os.getpid(), signal.SIGINT)
                    break
                if ch == b"\x1b":
                    rest = sys.stdin.buffer.read(2)
                    seq = {b"[A": "UP", b"[B": "DOWN", b"[C": "RIGHT", b"[D": "LEFT"}.get(rest)
                    if seq and seq in callbacks:
                        callbacks[seq]()
                    continue
                key = ch.decode("utf-8", errors="ignore").upper()
                if key in callbacks:
                    callbacks[key]()
        except Exception:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)

    threading.Thread(target=_loop, daemon=True).start()


class SpotRunner:
    def __init__(self, physics_dt, render_dt, obs_mode: str = "loco", policy_path: str | None = None,
                 log_csv: str | None = None, gain_switch_s: float = 0.0,
                 grasp_object: str = "SM_BottlePlasticB_01") -> None:
        self._world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt)
        self._phase2_config = load_spot_loco_phase2_config()

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")

        # Custom warehouse: simple_warehouse + real table + clutter objects
        # pre-arranged on top (drill, jar, etc.)
        prim = define_prim("/World/Warehouse", "Xform")
        prim.GetReferences().AddReference(WAREHOUSE_USD)

        base_dir = Path(__file__).resolve().parent.parent
        policy_params_path = os.path.join(base_dir, "params", "env.yaml")
        urdf_path = str(Path(base_dir) / "robot" / "spot" / "spot_with_arm.urdf")
        _import_spot_from_urdf(urdf_path=urdf_path, prim_path="/World/Spot")

        if obs_mode == "loco":
            policy_pt = policy_path or str(
                Path(base_dir) / "policies" / "spot_warehouse_policy.pt"
            )
        else:
            policy_pt = str(Path(base_dir) / "policies" / "spot_arm_policy.pt")
        self._spot = SpotLocoPolicy(
            prim_path="/World/Spot",
            name="Spot",
            usd_path=None,
            policy_path=policy_pt,
            policy_params_path=policy_params_path,
            position=np.array([-2.5, 0.0, 0.7]),
            physics_dt=physics_dt,
            phase2_config=self._phase2_config,
            obs_mode=obs_mode,
        )
        self._trained_gains_applied = False
        self._arm_poses = load_csv_poses(
            self._phase2_config["csv_path"],
            self._phase2_config["policy_arm_order"],
            self._phase2_config["csv_joint_cols"],
        )
        self._active_difficulty = "easy"
        self._arm_sequence = list(self._phase2_config["difficulty_groups"][self._active_difficulty])
        self._pose_idx = 0
        self._arm_routine_idx = -1
        self._arm_ticks = 0
        self._policy_hz = int(round(1.0 / (physics_dt * self._spot._decimation)))
        self._pose_hold_ticks = int(self._phase2_config["arm_tracking"]["pose_hold_s"] * self._policy_hz)
        self._default_hold_ticks = int(self._phase2_config["arm_tracking"]["default_hold_s"] * self._policy_hz)

        self._cameras = create_spot_cameras("/World/Spot", ALL_CAMERAS)
        self._ros_bridge = None
        self._arm_sub = SpotArmCommandSubscriber(self._spot)

        # --- arm gain experiment ---
        self._physics_dt = physics_dt
        self._gain_switch_s = float(gain_switch_s)
        self._gain_sweep = ARM_GAIN_SWEEP
        self._gain_idx = 0  # current index into the sweep
        self._gain_last_switch_t = 0.0
        self._sim_t = 0.0
        self._arm_logger = ArmDataLogger(log_csv, ARM_JOINTS) if log_csv else None
        self._arm_logger_ready = False
        # Physics-backed handles for the grasped object + end-effector, resolved
        # on the first physics step (USD xforms are stale under Fabric, so these
        # must read from the physics sim, not the stage).
        # Which object to grasp/log/stabilize. Accept either a full prim path or a
        # short name resolved under the warehouse stage, so it's no longer hardcoded.
        self._grasp_obj_path = (
            grasp_object if grasp_object.startswith("/")
            else f"/World/Warehouse/{grasp_object}"
        )
        self._ee_link_name = "arm_link_wr1"  # hand/palm; stable (the finger moves)
        self._grasp_obj = None
        self._ee_link_idx = None  # link index into the articulation (read-only, non-invasive)

        self._vx_sens = 1.5
        self._vy_sens = 0.8
        self._wz_sens = 1.5
        self._base_command = np.zeros(3)
        self.needs_reset = False
        self.first_step = True
        self._bridge_delay = 0

        # Object-only reset: snapshot each warehouse rigid body's spawn pose so a
        # keypress can restore the clutter without resetting Spot.
        self._object_reset_requested = False
        self._object_handles = {}        # prim path -> SingleRigidPrim
        self._object_initial_state = {}  # prim path -> (pos[3], quat_wxyz[4])

    def _set_vel(self, vx: float, vy: float, wz: float) -> None:
        self._base_command = np.array([vx, vy, wz])
        print(f"[Vel] cmd=[{vx:.1f}, {vy:.1f}, {wz:.1f}]")

    def _set_arm_pose_from_sequence(self, idx_in_sequence: int) -> None:
        pose_idx = self._arm_sequence[idx_in_sequence % len(self._arm_sequence)]
        self._pose_idx = pose_idx
        self._spot.set_arm_goal(self._arm_poses[pose_idx])
        print(
            f"[ArmPose] -> pose {pose_idx:3d} "
            f"({self._active_difficulty}, {idx_in_sequence % len(self._arm_sequence) + 1}/{len(self._arm_sequence)})"
        )

    def _advance_arm_routine(self) -> None:
        self._arm_routine_idx = (self._arm_routine_idx + 1) % len(self._arm_sequence)
        self._set_arm_pose_from_sequence(self._arm_routine_idx)
        self._arm_ticks = 0

    def _set_default_arm(self) -> None:
        self._arm_routine_idx = -1
        self._arm_ticks = 0
        self._spot.set_default_arm_pose()
        print("[ArmPose] -> default (stow)")

    def _scale_velocity(self, factor: float) -> None:
        self._vx_sens = round(self._vx_sens * factor, 3)
        self._vy_sens = round(self._vy_sens * factor, 3)
        self._wz_sens = round(self._wz_sens * factor, 3)
        print(f"[Vel] sens -> vx={self._vx_sens:.2f}  vy={self._vy_sens:.2f}  wz={self._wz_sens:.2f}")

    def _switch_difficulty(self, name: str) -> None:
        self._active_difficulty = name
        self._arm_sequence = list(self._phase2_config["difficulty_groups"][name])
        self._arm_routine_idx = -1
        self._arm_ticks = 0
        print(f"[Difficulty] -> {name.upper()} ({len(self._arm_sequence)} poses)")

    def setup(self) -> None:
        callbacks = {
            "UP": lambda: self._set_vel(self._vx_sens, 0.0, 0.0),
            "DOWN": lambda: self._set_vel(-self._vx_sens, 0.0, 0.0),
            "LEFT": lambda: self._set_vel(0.0, self._vy_sens, 0.0),
            "RIGHT": lambda: self._set_vel(0.0, -self._vy_sens, 0.0),
            "Z": lambda: self._set_vel(0.0, 0.0, self._wz_sens),
            "X": lambda: self._set_vel(0.0, 0.0, -self._wz_sens),
            "L": lambda: self._set_vel(0.0, 0.0, 0.0),
            "N": lambda: self._set_arm_pose_from_sequence(
                self._arm_sequence.index(self._pose_idx) + 1 if self._pose_idx in self._arm_sequence else 0
            ),
            "P": lambda: self._set_arm_pose_from_sequence(
                self._arm_sequence.index(self._pose_idx) - 1 if self._pose_idx in self._arm_sequence else 0
            ),
            "0": self._set_default_arm,
            "+": lambda: self._scale_velocity(1.2),
            "-": lambda: self._scale_velocity(1 / 1.2),
            "E": lambda: self._switch_difficulty("easy"),
            "M": lambda: self._switch_difficulty("medium"),
            "H": lambda: self._switch_difficulty("hard"),
            "R": self._request_object_reset,
            "T": self._request_full_reset,
        }
        _start_keyboard_listener(callbacks)
        print("[Keyboard] terminal raw-stdin listener started.")
        print("  Arrow Up/Down  -> forward / backward")
        print("  Arrow Left/Right -> strafe left / right")
        print("  Z / X          -> yaw left / right")
        print("  L              -> stop (zero velocity)")
        print("  N / P          -> next / previous arm pose")
        print("  0              -> arm to default stow")
        print("  + / -          -> scale all velocity sens x1.2 / x0.83")
        print("  E / M / H      -> easy / medium / hard arm sets")
        print("  R              -> reset all warehouse objects (Spot untouched)")
        print("  T              -> full reset (Spot to spawn + objects + re-init)")
        print("  Ctrl-C         -> quit")

        self._world.add_physics_callback("spot_forward", callback_fn=self.on_physics_step)

    def _request_object_reset(self) -> None:
        """Flag an object-only reset; the actual reset runs in the physics step
        (setting poses from the keyboard thread is not physics-safe)."""
        self._object_reset_requested = True
        print("[ObjReset] requested — will restore warehouse objects on next step")

    def _request_full_reset(self) -> None:
        """Flag a full world reset: Spot returns to its spawn pose and the whole
        scene re-initializes (same path used when the sim is stopped/restarted).
        This resets Spot AND the objects, and re-creates the ROS bridge/cameras.
        The flag is consumed in on_physics_step (reset is not thread-safe here)."""
        self.needs_reset = True
        print("[FullReset] requested — Spot to spawn + scene re-init on next step")

    def _capture_object_initial_state(self) -> None:
        """Snapshot the spawn pose of every dynamic rigid body under the
        warehouse so they can later be restored without touching Spot."""
        try:
            from pxr import UsdPhysics
            import omni.usd
            from isaacsim.core.prims import SingleRigidPrim
        except Exception as exc:
            print(f"[ObjReset] import failed, object reset disabled: {exc}")
            return

        stage = omni.usd.get_context().get_stage()
        paths = [
            prim.GetPath().pathString
            for prim in stage.Traverse()
            if prim.GetPath().pathString.startswith("/World/Warehouse")
            and prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ]
        for i, path in enumerate(paths):
            try:
                rp = SingleRigidPrim(prim_path=path, name=f"obj_reset_{i}")
                try:
                    rp.initialize()
                except Exception:
                    pass  # already initialized by the world reset
                pos, quat = rp.get_world_pose()
                self._object_handles[path] = rp
                self._object_initial_state[path] = (np.asarray(pos), np.asarray(quat))
            except Exception as exc:
                print(f"[ObjReset] could not capture {path}: {exc}")
        print(f"[ObjReset] captured spawn pose of {len(self._object_handles)} object(s)")

    def _reset_objects(self) -> None:
        """Restore every captured object to its spawn pose with zero velocity."""
        n = 0
        for path, rp in self._object_handles.items():
            pos, quat = self._object_initial_state[path]
            try:
                rp.set_world_pose(position=pos, orientation=quat)
                rp.set_linear_velocity(np.zeros(3))
                rp.set_angular_velocity(np.zeros(3))
                n += 1
            except Exception as exc:
                print(f"[ObjReset] reset failed on {path}: {exc}")
        print(f"[ObjReset] restored {n} object(s) to spawn pose")

    def _init_grasp_handles(self) -> float:
        """Bind the grasped object as a rigid prim and resolve the EE link index.

        The object is a standalone rigid body, so a SingleRigidPrim handle is
        safe. The EE link is part of the robot articulation — we must NOT wrap it
        as its own rigid body (that splits it from the articulation and detaches
        the finger). We only record its link index and READ its pose from the
        articulation's existing physics view in _read_grasp_state.

        Returns the object mass (kg), or NaN if unavailable. Best-effort.
        """
        obj_mass = float("nan")
        try:
            from isaacsim.core.prims import SingleRigidPrim
            self._grasp_obj = SingleRigidPrim(prim_path=self._grasp_obj_path, name="grasp_obj")
            try:
                self._grasp_obj.initialize()
            except Exception:
                pass  # already initialized by the world reset
            obj_mass = float(self._grasp_obj.get_mass())
            print(f"[GraspLog] object {self._grasp_obj_path} mass={obj_mass:.3f} kg")
        except Exception as exc:
            print(f"[GraspLog] could not bind object {self._grasp_obj_path}: {exc}")
            self._grasp_obj = None

        # EE link index only — never a separate rigid body.
        try:
            self._ee_link_idx = self._spot.robot._articulation_view.get_link_index(self._ee_link_name)
            print(f"[GraspLog] EE link '{self._ee_link_name}' index={self._ee_link_idx} (read-only)")
        except Exception as exc:
            print(f"[GraspLog] could not resolve EE link '{self._ee_link_name}': {exc}")
            self._ee_link_idx = None
        return obj_mass

    def _stabilize_grasp_physics(self) -> None:
        """Make the squeeze grasp stable: high friction on the object + gripper
        pads, no restitution, and more solver iterations so the pressed object
        doesn't penetrate/jitter/slip. All best-effort and non-invasive (material
        binding + solver-count properties, never a new rigid body).
        """
        try:
            import omni.usd
            from pxr import UsdPhysics, PhysxSchema
            from isaacsim.core.api.materials import PhysicsMaterial
            from isaacsim.core.prims import SingleGeometryPrim
        except Exception as exc:
            print(f"[GraspPhys] import failed, skipping stabilization: {exc}")
            return

        # 1) high-friction, zero-restitution material
        mat = None
        try:
            mat = PhysicsMaterial(
                prim_path="/World/Physics/grasp_high_friction",
                name="grasp_high_friction",
                static_friction=1.4, dynamic_friction=1.2, restitution=0.0,
            )
        except Exception as exc:
            print(f"[GraspPhys] material create failed: {exc}")

        stage = omni.usd.get_context().get_stage()

        # Collect every dynamic rigid body under the warehouse — these are the
        # grasp candidates. Static colliders (floor/walls/table) carry no
        # RigidBodyAPI so they're excluded automatically; the robot lives under
        # /World/Spot and is excluded by the path prefix. This makes the grasp
        # physics work for ANY object you reach for, not just one hardcoded prim.
        grasp_objects = [
            prim.GetPath().pathString
            for prim in stage.Traverse()
            if prim.GetPath().pathString.startswith("/World/Warehouse")
            and prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ]
        print(f"[GraspPhys] {len(grasp_objects)} dynamic rigid body(ies) under /World/Warehouse")

        # 2) bind the friction material to every grasp candidate's collider
        if mat is not None:
            applied = 0
            for path in grasp_objects:
                try:
                    SingleGeometryPrim(prim_path=path).apply_physics_material(mat)
                    applied += 1
                except Exception as exc:
                    print(f"[GraspPhys] material apply failed on {path}: {exc}")
            print(f"[GraspPhys] friction material on {applied}/{len(grasp_objects)} object(s)")

            # 3) bind it to the gripper pads (finger + fixed jaw colliders)
            pads = 0
            for prim in stage.Traverse():
                path = prim.GetPath().pathString
                low = path.lower()
                if prim.HasAPI(UsdPhysics.CollisionAPI) and ("fngr" in low or "jaw" in low):
                    try:
                        SingleGeometryPrim(prim_path=path).apply_physics_material(mat)
                        pads += 1
                    except Exception:
                        pass
            print(f"[GraspPhys] friction material on {pads} gripper pad collider(s)")

        # 4) more solver iterations on the robot articulation (resolves the squeeze)
        try:
            av = self._spot.robot._articulation_view
            av.set_solver_position_iteration_counts(np.array([32]))
            av.set_solver_velocity_iteration_counts(np.array([4]))
            print("[GraspPhys] robot solver iters -> pos=32 vel=4")
        except Exception as exc:
            print(f"[GraspPhys] robot solver iter set failed: {exc}")

        # 5) more solver iterations on every grasp object too
        objs_done = 0
        for path in grasp_objects:
            try:
                obj_prim = stage.GetPrimAtPath(path)
                rb = PhysxSchema.PhysxRigidBodyAPI.Apply(obj_prim)
                rb.CreateSolverPositionIterationCountAttr(32)
                rb.CreateSolverVelocityIterationCountAttr(4)
                objs_done += 1
            except Exception as exc:
                print(f"[GraspPhys] object solver iter set failed on {path}: {exc}")
        print(f"[GraspPhys] object solver iters -> pos=32 vel=4 on {objs_done} body(ies)")

    @staticmethod
    def _to_numpy(x):
        """Convert a warp/torch/np array to numpy without assuming the backend."""
        if x is None:
            return None
        if hasattr(x, "numpy"):
            try:
                return x.numpy()
            except Exception:
                pass
        if hasattr(x, "detach"):
            try:
                return x.detach().cpu().numpy()
            except Exception:
                pass
        return np.asarray(x)

    def _read_grasp_state(self):
        """Return (obj_pos[3], obj_vel[3], ee_pos[3]) world-frame, NaN on failure.

        EE pose is read non-invasively from the articulation's physics view link
        transforms — no extra rigid body is created.
        """
        obj_pos = obj_vel = ee_pos = None
        if self._grasp_obj is not None:
            try:
                obj_pos, _ = self._grasp_obj.get_world_pose()
                obj_vel = self._grasp_obj.get_linear_velocity()
            except Exception:
                obj_pos = obj_vel = None
        if self._ee_link_idx is not None:
            try:
                pv = self._spot.robot._articulation_view._physics_view
                xf = np.asarray(self._to_numpy(pv.get_link_transforms()))  # pos3 + quat4
                # shape is (num_envs, num_links, 7) for a view, or (num_links, 7)
                link_row = xf[0, self._ee_link_idx] if xf.ndim == 3 else xf[self._ee_link_idx]
                ee_pos = link_row[:3]
            except Exception:
                ee_pos = None
        return obj_pos, obj_vel, ee_pos

    def on_physics_step(self, step_size) -> None:
        if self.first_step:
            self._spot.initialize()
            self._spot.ensure_joint_ordering_ready()
            if not self._trained_gains_applied:
                apply_trained_gains(self._spot.robot, self._phase2_config)
                self._trained_gains_applied = True
            # only override gains when a sweep is requested
            if self._gain_switch_s > 0.0:
                name, gains = self._gain_sweep[self._gain_idx]
                apply_arm_gains(self._spot.robot, gains)
                print(f"[ArmGains] starting sweep at preset[{self._gain_idx}] = {name}")
            else:
                print("[ArmGains] no sweep — using gains from spot_loco_phase2.yaml")
            obj_mass = self._init_grasp_handles()
            self._stabilize_grasp_physics()
            self._capture_object_initial_state()
            if self._arm_logger is not None:
                policy_arm_order = self._phase2_config.get("policy_arm_order", ARM_JOINTS)
                self._arm_logger.resolve_indices(
                    self._spot.robot, policy_arm_order,
                    extra_meta={"object_path": self._grasp_obj_path,
                                "object_mass_kg": obj_mass,
                                "ee_link": self._ee_link_name},
                )
                self._arm_logger_ready = True
                print(f"[ArmLogger] writing CSV: {self._arm_logger._path}")
            self._set_default_arm()
            initialize_cameras(list(self._cameras.values()), enable_depth=True)
            self._arm_sub.start()
            self.first_step = False
            self._bridge_delay = 120  # wait ~120 physics steps (0.6 s) for render products to be ready
        elif self._bridge_delay > 0:
            self._bridge_delay -= 1
            if self._bridge_delay == 0:
                self._ros_bridge = ROSBridgeBuilder(
                    robot_prim_path="/World/Spot",
                    articulation_root_path="/World/Spot/body",
                    cameras=self._cameras,
                )
                if self._ros_bridge.success:
                    logger.info("ROS2 bridge ready — cameras publishing at ~10 Hz")
                else:
                    logger.warning("ROS2 bridge failed: %s", self._ros_bridge.error)
        elif self.needs_reset:
            self._world.reset(True)
            self.needs_reset = False
            self.first_step = True
            self._bridge_delay = 0
            self._ros_bridge = None
        else:
            if self._object_reset_requested:
                self._reset_objects()
                self._object_reset_requested = False
            if self._spot._policy_counter % self._spot._decimation == 0:
                if self._arm_sub.arm_override:
                    self._arm_ticks = 0
                else:
                    hold_ticks = self._default_hold_ticks if self._arm_routine_idx == -1 else self._pose_hold_ticks
                    if self._arm_ticks >= hold_ticks:
                        self._advance_arm_routine()
                    self._arm_ticks += 1
            self._spot.forward(step_size, self._base_command)
            self._arm_sub.update(step_size)
            if self._arm_sub.arm_override:
                action = self._arm_sub.get_arm_action()
                if action is not None:
                    self._spot.robot.apply_action(action)

            # advance sim time, rotate through gain sweep, log arm state
            self._sim_t += step_size
            if self._gain_switch_s > 0.0 and (self._sim_t - self._gain_last_switch_t) >= self._gain_switch_s:
                self._gain_idx = (self._gain_idx + 1) % len(self._gain_sweep)
                name, gains = self._gain_sweep[self._gain_idx]
                apply_arm_gains(self._spot.robot, gains)
                self._gain_last_switch_t = self._sim_t
                print(f"[ArmGains] t={self._sim_t:.2f}s -> preset[{self._gain_idx}] = {name}")
            if self._arm_logger_ready:
                # Prefer the live ROS/curobo commanded target when override is active,
                # otherwise fall back to the policy's arm_goal_policy.
                if self._arm_sub.arm_override and self._arm_sub._arm_position is not None:
                    target_array = self._arm_sub._arm_position
                    tgt_source = "ros"
                else:
                    target_array = getattr(self._spot, "arm_goal_policy", None)
                    tgt_source = "policy"
                preset_name = self._gain_sweep[self._gain_idx][0]
                # Replicate spot_policy._compute_observation's base-frame transform so
                # logged base state matches what the policy actually sees.
                lin_vel_i = self._spot.robot.get_linear_velocity()
                ang_vel_i = self._spot.robot.get_angular_velocity()
                _, q_ib = self._spot.robot.get_world_pose()
                r_bi = quat_to_rot_matrix(q_ib).T
                base_lin_vel_b = r_bi @ lin_vel_i
                base_ang_vel_b = r_bi @ ang_vel_i
                projected_gravity_b = r_bi @ np.array([0.0, 0.0, -1.0])
                obj_pos, obj_vel, ee_pos = self._read_grasp_state()
                self._arm_logger.log(
                    step_size,
                    preset_name,
                    self._spot.robot,
                    target_array,
                    tgt_source,
                    pose_idx=self._pose_idx if self._arm_routine_idx != -1 else -1,
                    difficulty=self._active_difficulty if self._arm_routine_idx != -1 else "default",
                    cmd=self._base_command,
                    base_lin_vel_b=base_lin_vel_b,
                    base_ang_vel_b=base_ang_vel_b,
                    projected_gravity_b=projected_gravity_b,
                    obj_pos=obj_pos,
                    obj_vel=obj_vel,
                    ee_pos=ee_pos,
                )

    def run(self) -> None:
        while simulation_app.is_running():
            self._world.step(render=True)
            if self._world.is_stopped():
                self.needs_reset = True


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--obs-mode", choices=["loco", "arm"], default="loco",
                        help="loco: locomanipulation policy (default); arm: original spot_arm_policy")
    parser.add_argument("--policy", type=str, default=None, help="Path to policy .pt file (loco mode only)")
    parser.add_argument("--log-csv", type=str, default=None,
                        help="Path to CSV for per-step arm joint logging (target/pos/vel + gain_mode). "
                             "Pass 'auto' to write to /tmp/arm_gain_log_<timestamp>.csv")
    parser.add_argument("--gain-switch-s", type=float, default=0.0,
                        help="Toggle arm gains between ORIG and NEW every N seconds (0 = no switching)")
    parser.add_argument("--grasp-object", type=str, default="SM_BottlePlasticB_01",
                        help="Object to grasp/log/stabilize: a short name resolved under "
                             "/World/Warehouse, or a full prim path. The high-friction grasp "
                             "material and per-step obj_* logging bind to this prim.")
    args, _ = parser.parse_known_args()

    log_csv = args.log_csv
    if log_csv == "auto":
        log_csv = f"/tmp/arm_gain_log_{int(time.time())}.csv"

    physics_dt = 1 / 200.0
    render_dt = 1 / 60.0

    runner = SpotRunner(
        physics_dt=physics_dt,
        render_dt=render_dt,
        obs_mode=args.obs_mode,
        policy_path=args.policy,
        log_csv=log_csv,
        gain_switch_s=args.gain_switch_s,
        grasp_object=args.grasp_object,
    )
    simulation_app.update()
    runner._world.reset()
    simulation_app.update()
    runner.setup()
    simulation_app.update()
    try:
        runner.run()
    finally:
        if runner._arm_logger is not None:
            runner._arm_logger.close()
    runner._world.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
