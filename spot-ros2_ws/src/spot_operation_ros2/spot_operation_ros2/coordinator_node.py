import json
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class CoordinatorNode(Node):
    """
    Lightweight state machine:
    - Reads tracking state.
    - Requests relocalization via VLM service when needed.
    - Publishes the seed command for the SAM 2 tracker node.
    """

    def __init__(self):
        super().__init__("coordinator_node")
        self.declare_parameter("tracking_state_topic", "/tracking_state")
        self.declare_parameter("seed_command_topic", "/perception/seed_command")
        self.declare_parameter("vlm_service_name", "/roll/trigger_relocalize")
        self.declare_parameter("loop_hz", 2.0)
        self.declare_parameter("max_seed_age_sec", 10.0)
        self.declare_parameter("relocalize_cooldown_sec", 1.5)
        self.declare_parameter("seed_apply_grace_sec", 2.0)
        self.declare_parameter("disable_auto_relocalize_on_lost", True)

        tracking_state_topic = self.get_parameter("tracking_state_topic").value
        seed_command_topic = self.get_parameter("seed_command_topic").value
        service_name = self.get_parameter("vlm_service_name").value
        loop_hz = float(max(0.5, self.get_parameter("loop_hz").value))
        self.max_seed_age_sec = float(max(1.0, self.get_parameter("max_seed_age_sec").value))
        self.relocalize_cooldown_sec = float(
            max(0.0, self.get_parameter("relocalize_cooldown_sec").value)
        )
        self.seed_apply_grace_sec = float(
            max(0.0, self.get_parameter("seed_apply_grace_sec").value)
        )
        self.disable_auto_relocalize_on_lost = self.get_parameter("disable_auto_relocalize_on_lost").value
        self._use_sim_time_effective = bool(self.get_parameter("use_sim_time").value)

        self.state = "IDLE"
        self.tracking_state = "UNKNOWN"
        self._vlm_inflight = False
        self._last_relocalize_attempt = 0.0
        self._seed_apply_deadline = 0.0
        self._current_prompt_change_id = 0
        self._tracking_sub = self.create_subscription(
            String, tracking_state_topic, self._tracking_state_cb, 10
        )
        self._seed_pub = self.create_publisher(String, seed_command_topic, 10)
        self._state_pub = self.create_publisher(String, "/coordinator/state", 10)
        self._vlm_client = self.create_client(Trigger, service_name)
        self._timer = self.create_timer(1.0 / loop_hz, self._tick)
        self._prompt_change_sub = self.create_subscription(
            String, "/vlm/prompt_change_id", self._prompt_change_cb, 10
        )
        self.get_logger().info(
            f"Coordinator started. tracking_state={tracking_state_topic}, seed_out={seed_command_topic}, vlm_service={service_name}"
        )

    def _tracking_state_cb(self, msg: String):
        self.tracking_state = msg.data.strip().upper()
        if self.tracking_state == "TRACKING":
            # Tracker locked on target: clear grace window.
            self._seed_apply_deadline = 0.0


    def _prompt_change_cb(self, msg: String):
        """Handle prompt change notifications from VLM node."""
        new_id = int(msg.data)
        if new_id > self._current_prompt_change_id:
            self._current_prompt_change_id = new_id
            self.get_logger().info(f"[COORD] Prompt change detected (id={new_id}), triggering VLM")
            self._request_vlm_relocalize()

    def _request_vlm_relocalize(self):
        """Trigger VLM relocalization request."""
        now = time.time()
        if not self._vlm_client.wait_for_service(timeout_sec=0.0):
            self.state = "DEGRADED"
            self.get_logger().warn("[COORD] VLM service unavailable")
            self._publish_state()
            return
        self._last_relocalize_attempt = now
        self._vlm_inflight = True
        self.state = "RELOCALIZING"
        self._publish_state()
        self.get_logger().info(f"[COORD] Triggering VLM (prompt change or manual)")
        fut = self._vlm_client.call_async(Trigger.Request())
        fut.add_done_callback(self._handle_vlm_result)

    def _publish_state(self):
        state_msg = String()
        state_msg.data = self.state
        self._state_pub.publish(state_msg)
    def _tick(self):
        now = time.time()
        if self.tracking_state == "TRACKING":
            self.state = "TRACKING"
            return
        if now < self._seed_apply_deadline:
            self.state = "SEEDING"
            return
        # Trigger relocalization only when explicitly LOST.
        # UNKNOWN/IDLE/other states should not spam API calls.
        if self.tracking_state != "LOST":
            self.state = "IDLE"
            return
        # If disabled, don't auto-trigger on lost (let SAM2's memory handle re-detection)
        if self.disable_auto_relocalize_on_lost:
            self.state = "IDLE"
            self._publish_state()
            return
        if self._vlm_inflight:
            self.state = "RELOCALIZING"
            return
        if now - self._last_relocalize_attempt < self.relocalize_cooldown_sec:
            return
        if not self._vlm_client.wait_for_service(timeout_sec=0.0):
            self.state = "DEGRADED"
            return
        self._last_relocalize_attempt = now
        self._vlm_inflight = True
        self.state = "RELOCALIZING"
        fut = self._vlm_client.call_async(Trigger.Request())
        fut.add_done_callback(self._handle_vlm_result)

    def _handle_vlm_result(self, future):
        self._vlm_inflight = False
        try:
            result = future.result()
        except Exception as exc:
            self.state = "DEGRADED"
            self.get_logger().warn(f"Relocalize call failed: {exc}")
            return
        if result is None or not result.success:
            self.state = "DEGRADED"
            return
        # Seed is published by vlm_relocalize_node directly on /perception/seed_command,
        # so the manual service path also drives the full pipeline. Coordinator only
        # tracks the grace window and state here.
        self._seed_apply_deadline = time.time() + self.seed_apply_grace_sec
        self.state = "IDLE"


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

