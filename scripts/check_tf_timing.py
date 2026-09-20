#!/usr/bin/env python3
"""Read-only timing audit. Sources must be running; never publishes robot commands."""
import argparse
import json
import math
import time
from pathlib import Path
from collections import defaultdict

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Bool, String
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--strict", action="store_true", help="Check proposed TF commissioning limits; does not certify navigation safety")
    parser.add_argument("--output", help="Save the same JSON report to a file")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("duration must be positive")
    rclpy.init()
    node = rclpy.create_node("nav_ws_tf_timing_audit")
    ages = defaultdict(list)
    metrics = defaultdict(list)
    subscriptions = []
    health = defaultdict(list)
    reasons = defaultdict(lambda: defaultdict(int))
    latest = {}
    lookup_waits = defaultdict(list)
    lookup_failures = defaultdict(int)
    pending = []
    buffer = Buffer()
    listener = TransformListener(buffer, node)


    def record(key, stamp):
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        latest[key] = stamp_ns
        ages[key].append((node.get_clock().now().nanoseconds - stamp_ns) / 1e6)

    def tf_callback(message):
        for transform in message.transforms:
            if (transform.header.frame_id, transform.child_frame_id) in (
                ("map", "odom"), ("odom", "base_link"), ("camera_init", "body")
            ):
                record(f"TF {transform.header.frame_id} -> {transform.child_frame_id}", transform.header.stamp)

    subscriptions.append(node.create_subscription(TFMessage, "/tf", tf_callback, qos_profile_sensor_data))
    for topic, msg_type in (("/Odometry", Odometry), ("/odom", Odometry), ("/cloud_obstacles", PointCloud2)):
        subscriptions.append(node.create_subscription(
            msg_type, topic, lambda msg, key=topic: record(key, msg.header.stamp), qos_profile_sensor_data))
    for topic in ("/fastlio_odom_bridge/input_age_ms", "/fastlio_odom_bridge/processing_ms",
                  "/localization_3d_registration_age_ms", "/localization_3d_registration_duration_ms",
                  "/localization_3d_confidence", "/odom_prediction/anchor_age_ms",
                  "/odom_prediction/imu_age_ms", "/odom_prediction/processing_ms",
                  "/collision_monitor/processing_ms", "/collision_monitor/speed_scale",
                  "/collision_monitor/effective_reaction_s", "/odom_prediction/correction_position_m",
                  "/odom_prediction/correction_rotation_rad"):
        subscriptions.append(node.create_subscription(
            Float32, topic, lambda msg, key=topic: metrics[key].append(msg.data), qos_profile_sensor_data))
    for topic in ("/odom_prediction/valid", "/collision_monitor/ready"):
        subscriptions.append(node.create_subscription(
            Bool, topic, lambda msg, key=topic: health[key].append(msg.data), qos_profile_sensor_data))
    def reason_callback(key, msg):
        reasons[key][msg.data] += 1
    for topic in ("/odom_prediction/reason", "/collision_monitor/reason"):
        subscriptions.append(node.create_subscription(
            String, topic, lambda msg, key=topic: reason_callback(key, msg), qos_profile_sensor_data))
    held_ages = defaultdict(list)
    start = time.monotonic()
    next_probe = start + 2.0  # Allow TF subscriptions to discover before scoring.

    while rclpy.ok() and time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.002)
        current = time.monotonic()
        if current >= next_probe:
            stamp = node.get_clock().now()
            for parent in ("odom", "map"):
                pending.append((parent, stamp, current))
            for key, stamp_ns in latest.items():
                if key.startswith("TF "):
                    held_ages[key].append((stamp.nanoseconds-stamp_ns)/1e6)
            next_probe = current + 0.1
        remaining = []
        for parent, stamp, began in pending:
            key = parent + " -> base_link at request time"
            try:
                buffer.lookup_transform(parent, "base_link", stamp)
                if current-began <= 0.05:
                    lookup_waits[key].append((current-began)*1000)
                else:
                    lookup_failures[key] += 1
            except TransformException:
                if current-began >= 0.05:
                    lookup_failures[key] += 1
                else:
                    remaining.append((parent, stamp, began))
        pending = remaining

    def summary(values):
        ordered = sorted(x for x in values if math.isfinite(x))
        if not ordered:
            return {"samples": 0}
        return {"samples": len(ordered), "median": ordered[len(ordered)//2],
                "p95": ordered[min(len(ordered)-1, math.ceil(len(ordered)*0.95)-1)],
                "max": ordered[-1], "min": ordered[0]}

    result = {"duration_s": time.monotonic() - start,
              "message_age_ms": {k: summary(v) for k, v in ages.items()},
              "diagnostic_values": {k: summary(v) for k, v in metrics.items()},
              "tf_age_at_periodic_query_ms": {k: summary(v) for k, v in held_ages.items()},
              "tf_lookup_budget_ms": 50,
              "tf_lookup_warmup_s": 2,
              "tf_lookup_wait_ms": {k: summary(v) for k, v in lookup_waits.items()},
              "tf_lookup_failures": dict(lookup_failures),
              "tf_lookups_pending_at_end": len(pending),
              "health": {k: {"samples": len(v), "valid": sum(v), "invalid": len(v)-sum(v)} for k,v in health.items()},
              "reasons": {k: dict(v) for k,v in reasons.items()},
              "note": "Receipt age includes scheduling and transport; missing topics are not healthy by default. Registration age -1 means no accepted result."}
    if args.strict:
        failures = []
        duration = result["duration_s"]
        if duration < 10:
            failures.append("Use at least 10 seconds for commissioning; 60 seconds recommended")
        for key in ("/odom", "TF odom -> base_link", "TF map -> odom"):
            stat = result["message_age_ms"].get(key, {})
            if stat.get("samples", 0)/duration < 40:
                failures.append(key + " rate below 40 Hz or absent")
            if stat.get("p95", float('inf')) > 20 or stat.get("max", float('inf')) > 50:
                failures.append(key + " receipt age exceeds P95 20 ms / maximum 50 ms")
        for parent in ("odom", "map"):
            key = parent + " -> base_link at request time"
            if not lookup_waits[key] or lookup_failures[key]:
                failures.append(key + " failed 50 ms lookup budget or has no successful samples")
        prediction = health.get("/odom_prediction/valid", [])
        if not prediction or not all(prediction):
            failures.append("Prediction validity absent or false during capture")
        confidence = metrics.get("/localization_3d_confidence", [])
        if not confidence or any(not math.isfinite(x) or x < 0.7 for x in confidence):
            failures.append("Localization confidence absent or below 0.7")
        cloud = result["message_age_ms"].get("/cloud_obstacles", {})
        if cloud.get("samples",0)/duration < 8 or cloud.get("max",float('inf')) > 300:
            failures.append("Obstacle cloud absent, below 8 Hz, or older than 300 ms")
        result["acceptance"] = {"passed": not failures, "failures": failures,
            "scope": "Proposed TF/data timing gate only. Requires initialized localization; does not verify pose accuracy or physical stopping distance."}
    formatted = json.dumps(result, indent=2, ensure_ascii=False)
    if args.output:
        target = Path(args.output)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(formatted + "\n")
    print(formatted)
    node.destroy_node()
    rclpy.shutdown()
    if args.strict:
        return 0 if result["acceptance"]["passed"] else 2
    return 0 if ages else 1


if __name__ == "__main__":
    raise SystemExit(main())
