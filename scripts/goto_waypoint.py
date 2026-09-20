#!/usr/bin/env python3
"""Send a named JSON waypoint to Nav2's NavigateToPose action."""
import argparse
import json
import math
from pathlib import Path
import signal
import time


def load_waypoints(path):
    data = json.loads(path.read_text())
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('不支持的 JSON 格式')
    if data.get('frame_id') != 'map' or not isinstance(data.get('waypoints'), dict):
        raise ValueError('需要 frame_id=map 和 waypoints 字典')
    if not isinstance(data.get('map_yaml'), str) or not data['map_yaml'].strip():
        raise ValueError('缺少 map_yaml')
    map_path = (path.resolve().parent / data['map_yaml']).resolve()
    if not map_path.is_file():
        raise ValueError(f'地图文件不存在：{map_path}')
    for name, point in data['waypoints'].items():
        if not name.strip() or not isinstance(point, dict):
            raise ValueError('点位名称或格式无效')
        for key in ('x', 'y', 'z', 'yaw_deg'):
            value = point.get(key)
            if type(value) not in (float, int) or not math.isfinite(value):
                raise ValueError(f'{name}.{key} 必须是有限数值')
    return data, map_path


def make_goal(point, stamp):
    from nav2_msgs.action import NavigateToPose
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = stamp
    goal.pose.pose.position.x = float(point['x'])
    goal.pose.pose.position.y = float(point['y'])
    goal.pose.pose.position.z = float(point['z'])
    yaw = math.radians(point['yaw_deg'] % 360)
    goal.pose.pose.orientation.z = math.sin(yaw / 2)
    goal.pose.pose.orientation.w = math.cos(yaw / 2)
    return goal


def navigate(point, map_path):
    import rclpy
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from rcl_interfaces.srv import GetParameters
    from rclpy.action import ActionClient
    from rclpy.signals import SignalHandlerOptions

    # Keep ROS alive on Ctrl+C long enough to cancel this client's goal.
    stop = []
    old_signals = {s: signal.signal(s, lambda *_: stop.append(True))
                   for s in (signal.SIGINT, signal.SIGTERM)}
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('goto_waypoint')
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    params = node.create_client(GetParameters, '/map_server/get_parameters')
    handle = None

    def wait(future, timeout):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('等待 ROS 响应超时')
        return future.result()

    def cancel():
        response = wait(handle.cancel_goal_async(), 5.0)
        result = wait(handle.get_result_async(), 5.0)
        if result.status == GoalStatus.STATUS_CANCELED:
            print('本次导航已取消。')
        elif result.status == GoalStatus.STATUS_SUCCEEDED:
            print('目标已到达。')
        else:
            print(f'导航已结束：status={result.status}，取消返回码={response.return_code}')

    try:
        if not params.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('未找到 map_server，请先启动导航')
        request = GetParameters.Request(names=['yaml_filename'])
        response = wait(params.call_async(request), 5.0)
        loaded = response.values[0].string_value if response.values else ''
        if not loaded or not Path(loaded).is_absolute():
            raise RuntimeError('无法确认当前地图绝对路径，未发送目标')
        if Path(loaded).resolve() != map_path:
            raise RuntimeError(f'地图不一致：当前 {loaded}，点位要求 {map_path}')
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('Nav2 NavigateToPose 服务未就绪')
        if stop:
            return 130
        last_feedback = [0.0]

        def feedback(message):
            now = time.monotonic()
            if now - last_feedback[0] >= 3.0:
                print(f'剩余距离：{message.feedback.distance_remaining:.2f} m', flush=True)
                last_feedback[0] = now

        pending = client.send_goal_async(make_goal(point, node.get_clock().now().to_msg()),
                                         feedback_callback=feedback)
        try:
            handle = wait(pending, 10.0)
        except RuntimeError:
            print('目标请求已发出，但未收到接收确认；状态未知，请在 RViz 检查或取消目标。')
            raise
        if not handle.accepted:
            handle = None
            raise RuntimeError('Nav2 拒绝了目标，请检查导航是否激活')
        print('Nav2 已接收目标。Ctrl+C 可请求取消本次导航。', flush=True)
        result = handle.get_result_async()
        while not result.done():
            if stop:
                cancel()
                return 130
            rclpy.spin_once(node, timeout_sec=0.2)
        status = result.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print('已到达目标点。')
            return 0
        print(f'导航未完成：{ {5: "已取消", 6: "失败"}.get(status, status) }')
        return 1
    except Exception as exc:
        print(f'导航错误：{exc}')
        if handle is not None:
            try:
                cancel()
            except Exception as cancel_error:
                print(f'无法确认取消结果：{cancel_error}；请在 RViz 检查或取消目标。')
        return 1
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name', nargs='?', help='JSON 中的点位名称')
    parser.add_argument('--file', type=Path, default=Path(__file__).resolve().parents[1]
                        / 'waypoints' / 'magic2_goals.json')
    parser.add_argument('--list', action='store_true', help='列出点位，不发送目标')
    parser.add_argument('--dry-run', action='store_true', help='只检查并显示目标，不连接 ROS')
    args = parser.parse_args()
    try:
        data, map_path = load_waypoints(args.file)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    if args.list:
        print(json.dumps(data['waypoints'], ensure_ascii=False, indent=2))
        if not data['waypoints']:
            print('暂无点位；先运行 ./save_waypoint.sh point_1 并在 RViz 选点。')
        return 0
    if not args.name or args.name not in data['waypoints']:
        parser.error('请指定已保存的点位名称；用 --list 查看')
    point = data['waypoints'][args.name]
    print(f'目标：{args.name}\n地图：{map_path}\n坐标：{point}', flush=True)
    return 0 if args.dry_run else navigate(point, map_path)


if __name__ == '__main__':
    raise SystemExit(main())
