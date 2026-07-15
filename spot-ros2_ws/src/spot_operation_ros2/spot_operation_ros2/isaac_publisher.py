#!/usr/bin/env python3
"""
Isaac Sim Joint Command Publisher

Publishes joint commands to Isaac Sim articulation controller.
Supports two modes via 'teleop' parameter:
  - teleop=false (default): reads from /arm_controller/controller_state
  - teleop=true: reads from /joint_command_curobo (cuRobo MPC)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float64


class JointStateRelay(Node):
    def __init__(self):
        super().__init__("joint_state_relay")

        # Publisher for joint_command_isaac
        self.pub = self.create_publisher(JointState, "joint_command_isaac", 10)

        # Store the last gripper command
        self.gripper_position = None

        # Teleop parameter to switch between modes
        self.declare_parameter("teleop", False)
        self.teleop_mode = self.get_parameter("teleop").get_parameter_value().bool_value

        if self.teleop_mode:
            # Teleop mode: subscribe to the cuRobo MPC
            self.sub = self.create_subscription(
                JointState,
                "/joint_command_curobo",
                self.curobo_callback,
                10,
            )
            self.get_logger().info("=== Isaac Publisher - TELEOP mode ===")
            self.get_logger().info("Subscribing to: /joint_command_curobo")
        else:
            # Default mode: subscribe to the arm_controller
            self.declare_parameter("input_topic", "/arm_controller/controller_state")
            input_topic = (
                self.get_parameter("input_topic").get_parameter_value().string_value
            )

            self.sub = self.create_subscription(
                JointTrajectoryControllerState,
                input_topic,
                self.joint_states_callback,
                10,
            )
            self.get_logger().info("=== Isaac Publisher - CONTROLLER mode ===")
            self.get_logger().info(f"Subscribing to: {input_topic}")

        # Subscriber for gripper commands (both modes)
        self.gripper_sub = self.create_subscription(
            Float64,
            "/gripper/command",
            self.gripper_command_callback,
            10,
        )

        self.get_logger().info("Subscribing to: /gripper/command")
        self.get_logger().info("Publishing to: joint_command_isaac")

    def curobo_callback(self, msg: JointState):
        """Callback for cuRobo MPC commands (teleop mode)."""
        # msg is already a JointState with arm0_* joints
        new_msg = JointState()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.name = list(msg.name)
        new_msg.position = list(msg.position)
        new_msg.velocity = list(msg.velocity) if msg.velocity else []
        new_msg.effort = list(msg.effort) if msg.effort else []

        # Append gripper if available
        if self.gripper_position is not None:
            if "arm0_f1x" not in new_msg.name:
                new_msg.name.append("arm0_f1x")
                new_msg.position.append(self.gripper_position)
                if new_msg.velocity:
                    new_msg.velocity.append(0.0)
                if new_msg.effort:
                    new_msg.effort.append(0.0)

        self.pub.publish(new_msg)

    def joint_states_callback(self, msg):
        """Callback for the arm_controller (default mode)."""
        # msg: control_msgs/msg/JointTrajectoryControllerState
        names = list(getattr(msg, "joint_names", []) or [])

        # Pick the best available position vector
        positions = None
        for attr in ("actual", "output", "feedback", "reference", "desired"):
            part = getattr(msg, attr, None)
            if part is None:
                continue
            if hasattr(part, "positions") and part.positions:
                positions = list(part.positions)
                break

        if not names or not positions:
            return

        # Keep only arm joints starting with "arm_" and rename them to "arm0_"
        arm_joints_indices = []
        new_names = []
        for i, joint_name in enumerate(names):
            if joint_name.startswith("arm_"):
                arm_joints_indices.append(i)
                new_names.append(joint_name.replace("arm_", "arm0_", 1))

        if not arm_joints_indices:
            return

        new_msg = JointState()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.name = new_names
        new_msg.position = [positions[i] for i in arm_joints_indices]

        # Try to fill velocity/effort if available
        vel = getattr(msg, "actual", None) and getattr(msg.actual, "velocities", None)
        if not vel:
            vel = getattr(msg, "output", None) and getattr(
                msg.output, "velocities", None
            )
        eff = getattr(msg, "actual", None) and getattr(msg.actual, "effort", None)

        new_msg.velocity = [vel[i] for i in arm_joints_indices] if vel else []
        new_msg.effort = [eff[i] for i in arm_joints_indices] if eff else []

        # Append gripper if available
        if self.gripper_position is not None:
            new_msg.name.append("arm0_f1x")
            new_msg.position.append(self.gripper_position)
            if new_msg.velocity:
                new_msg.velocity.append(0.0)
            if new_msg.effort:
                new_msg.effort.append(0.0)

        self.pub.publish(new_msg)

    def gripper_command_callback(self, msg: Float64):
        """Callback for gripper commands via /gripper/command"""
        try:
            self.gripper_position = msg.data
            self.get_logger().debug(f"Gripper command: {self.gripper_position}")
        except Exception as e:
            self.get_logger().warn(f"Error processing gripper command: {e}")


def main():
    rclpy.init()
    node = JointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
