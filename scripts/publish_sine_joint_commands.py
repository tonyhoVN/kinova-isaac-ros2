#!/usr/bin/env python3
"""Publish limit-aware sine position commands for every controllable joint."""

from __future__ import annotations

import argparse
import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = REPO_ROOT / "env/urdf/gen3.urdf"
DEFAULT_SRDF = REPO_ROOT / "env/urdf/gen3.srdf"
# Gen3's shoulder/elbow/wrist joints (joint_1/3/5/7) are URDF "continuous"
# joints with no position limit; bound their sine sweep to this range around
# the ready pose instead of a fraction of [lower, upper] (which doesn't exist).
CONTINUOUS_JOINT_HALF_RANGE_RAD = math.pi


@dataclass(frozen=True)
class SineJoint:
    name: str
    center: float
    amplitude: float


def load_ready_positions(srdf_path: Path) -> dict[str, float]:
    root = ET.parse(srdf_path).getroot()
    positions: dict[str, float] = {}
    for state in root.findall("./group_state[@name='ready']"):
        for joint in state.findall("joint"):
            name, value = joint.get("name"), joint.get("value")
            if name is not None and value is not None:
                positions[name] = float(value)
    return positions


def load_sine_joints(
    urdf_path: Path,
    srdf_path: Path,
    amplitude_scale: float,
    frequency: float,
) -> tuple[list[SineJoint], list[str]]:
    """Build safe trajectories from URDF limits and SRDF ready positions."""
    ready = load_ready_positions(srdf_path)
    root = ET.parse(urdf_path).getroot()
    trajectories: list[SineJoint] = []
    mimic_joints: list[str] = []
    angular_frequency = 2.0 * math.pi * frequency

    for joint in root.findall("joint"):
        joint_type = joint.get("type")
        if joint_type not in {"revolute", "prismatic", "continuous"}:
            continue
        name = joint.get("name")
        if name is None:
            continue
        if joint.find("mimic") is not None:
            mimic_joints.append(name)
            continue

        limit = joint.find("limit")
        lower_text = limit.get("lower") if limit is not None else None
        upper_text = limit.get("upper") if limit is not None else None
        if lower_text is not None and upper_text is not None:
            lower = float(lower_text)
            upper = float(upper_text)
        elif joint_type == "continuous":
            center_guess = ready.get(name, 0.0)
            lower = center_guess - CONTINUOUS_JOINT_HALF_RANGE_RAD
            upper = center_guess + CONTINUOUS_JOINT_HALF_RANGE_RAD
        else:
            raise RuntimeError(f"Joint {name!r} has no finite position limits")

        center = ready.get(name, (lower + upper) / 2.0)
        if not lower <= center <= upper:
            raise RuntimeError(
                f"Center {center} for {name!r} is outside [{lower}, {upper}]"
            )

        amplitude = amplitude_scale * min(center - lower, upper - center)
        velocity_limit = limit.get("velocity") if limit is not None else None
        if velocity_limit is not None and angular_frequency > 0.0:
            # Keep a 20% margin below the URDF velocity limit.
            amplitude = min(amplitude, 0.8 * float(velocity_limit) / angular_frequency)
        trajectories.append(SineJoint(name, center, amplitude))

    if not trajectories:
        raise RuntimeError(f"No controllable joints found in {urdf_path}")
    return trajectories, mimic_joints


def publish_center(publisher, node, joints: list[SineJoint], message_type) -> None:
    message = message_type()
    message.header.stamp = node.get_clock().now().to_msg()
    message.name = [joint.name for joint in joints]
    message.position = [joint.center for joint in joints]
    publisher.publish(message)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/isaac_joint_commands")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--srdf", type=Path, default=DEFAULT_SRDF)
    parser.add_argument("--rate", type=float, default=200.0, help="Publish rate in Hz")
    parser.add_argument(
        "--frequency", type=float, default=0.1, help="Sine frequency in Hz"
    )
    parser.add_argument(
        "--amplitude-scale",
        type=float,
        default=0.1,
        help="Fraction of available travel from the center (0 < value <= 1)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Stop after this many seconds; zero runs until Ctrl-C",
    )
    parser.add_argument(
        "--return-to-center",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Publish the ready/center pose once when stopping",
    )
    args = parser.parse_args()

    if args.rate <= 0.0:
        parser.error("--rate must be positive")
    if args.frequency <= 0.0:
        parser.error("--frequency must be positive")
    if not 0.0 < args.amplitude_scale <= 1.0:
        parser.error("--amplitude-scale must be in (0, 1]")
    if args.duration < 0.0:
        parser.error("--duration cannot be negative")
    if not args.urdf.is_file() or not args.srdf.is_file():
        parser.error("generated URDF/SRDF missing; run: python3 env/generate_urdf.py")

    joints, mimic_joints = load_sine_joints(
        args.urdf, args.srdf, args.amplitude_scale, args.frequency
    )

    import rclpy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node("gen3_sine_joint_publisher")
    publisher = node.create_publisher(JointState, args.topic, 10)
    period = 1.0 / args.rate
    angular_frequency = 2.0 * math.pi * args.frequency
    start = time.monotonic()
    next_publish = start

    node.get_logger().info(
        f"Publishing {len(joints)} sine joint commands to {args.topic} at "
        f"{args.rate:g} Hz; mimic joints follow automatically: {mimic_joints}"
    )
    try:
        while rclpy.ok():
            now = time.monotonic()
            elapsed = now - start
            if args.duration and elapsed >= args.duration:
                break
            if now < next_publish:
                time.sleep(next_publish - now)
                continue

            sine = math.sin(angular_frequency * elapsed)
            message = JointState()
            message.header.stamp = node.get_clock().now().to_msg()
            message.name = [joint.name for joint in joints]
            message.position = [
                joint.center + joint.amplitude * sine for joint in joints
            ]
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            next_publish += period
            if next_publish < now:
                next_publish = now + period
    except KeyboardInterrupt:
        pass
    finally:
        if args.return_to_center and rclpy.ok():
            publish_center(publisher, node, joints, JointState)
            time.sleep(0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
