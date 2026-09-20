#!/usr/bin/env python3
"""Record one planar RViz pose without publishing navigation commands."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import tempfile
import time


def save_pose(path, name, msg, replace=False):
    # Lock across read/modify/replace so simultaneous recorders preserve entries.
    with path.with_suffix('.json.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        data = json.loads(path.read_text())
        if data.get('schema_version') != 1 or not isinstance(data.get('waypoints'), dict):
            raise ValueError('不支持的点位文件格式')
        if msg.header.frame_id != data['frame_id']:
            raise ValueError(f"坐标系应为 {data['frame_id']}，收到 {msg.header.frame_id}")
        if name in data['waypoints'] and not replace:
            raise ValueError(f'点位 {name} 已存在；如需更新，请加 --replace')
        p, q = msg.pose.position, msg.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            raise ValueError('坐标或朝向不是有限数值')
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-8:
            raise ValueError('无效的朝向四元数')
        if abs(q.x / norm) > 1e-6 or abs(q.y / norm) > 1e-6:
            raise ValueError('仅支持平面点位，不能丢弃 roll/pitch')
        yaw = math.atan2(2*q.w*q.z, q.w*q.w - q.z*q.z)
        data['waypoints'][name] = {
            'x': p.x, 'y': p.y, 'z': p.z, 'yaw_deg': math.degrees(yaw)
        }
        temp_name = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=path.parent,
                                             prefix='.' + path.name, delete=False) as f:
                temp_name = f.name
                json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
                f.write('\n')
                f.flush()
                os.fsync(f.fileno())
            os.replace(temp_name, path)
        finally:
            if temp_name and os.path.exists(temp_name):
                os.unlink(temp_name)
        return data['waypoints'][name]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name', help='点位名称，例如 door_1')
    parser.add_argument('--file', type=Path, default=Path(__file__).resolve().parents[1]
                        / 'waypoints' / 'magic2_goals.json')
    parser.add_argument('--replace', action='store_true', help='允许替换同名点位')
    parser.add_argument('--timeout', type=float, default=120.0)
    args = parser.parse_args()
    if not args.name.strip() or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('名称不能为空，timeout 必须为正数')
    # Validate before asking the user to select a point.
    try:
        data = json.loads(args.file.read_text())
        if data.get('schema_version') != 1 or not isinstance(data.get('waypoints'), dict):
            raise ValueError('不支持的点位文件格式')
        if data.get('frame_id') != 'map':
            raise ValueError('点位文件 frame_id 必须为 map')
        if args.name in data['waypoints'] and not args.replace:
            raise ValueError('同名点位已存在；更新时加 --replace')
    except (OSError, ValueError) as exc:
        parser.error(str(exc))

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(args=[])
    node = rclpy.create_node('save_waypoint')
    received = []
    sub = node.create_subscription(PoseStamped, '/waypoint_pose', received.append,
                                   qos_profile_sensor_data)
    print('等待选点：RViz 的 2D Goal Pose，Topic 必须为 /waypoint_pose。', flush=True)
    print('请在 map 坐标系中点击并拖动设置朝向；Nav2 Goal 不用于此记录工具。', flush=True)
    deadline = time.monotonic() + args.timeout
    try:
        while not received and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not received:
            print('未收到选点，JSON 未更改。')
            return 1
        point = save_pose(args.file, args.name, received[0], args.replace)
        print(f'已保存 {args.name}: {point}\n文件：{args.file.resolve()}')
        return 0
    except (OSError, ValueError) as exc:
        print(f'保存失败：{exc}')
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
