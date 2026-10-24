#!/usr/bin/env python3
"""Send one NavigateToPose goal (map frame) and report the Nav2 result.

    python3 scripts/nav_goal.py 1.0 1.0 [--yaw 0] [--timeout 120]
"""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('x', type=float)
    p.add_argument('y', type=float)
    p.add_argument('--yaw', type=float, default=0.0)
    p.add_argument('--timeout', type=float, default=120.0, help='wall-clock seconds')
    args = p.parse_args()

    rclpy.init()
    node = Node('mazemind_nav_goal', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=30.0):
        print('navigate_to_pose action server not available', file=sys.stderr)
        return 2

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = args.x
    goal.pose.pose.position.y = args.y
    goal.pose.pose.orientation.z = math.sin(args.yaw / 2)
    goal.pose.pose.orientation.w = math.cos(args.yaw / 2)

    last = {'fb': None}
    send = client.send_goal_async(goal, feedback_callback=lambda m: last.update(fb=m.feedback))
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print('goal rejected')
        return 3
    result = handle.get_result_async()
    start = time.time()
    while not result.done() and time.time() - start < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
    if not result.done():
        handle.cancel_goal_async()
        print('TIMEOUT (wall clock)')
        return 4
    fb = last['fb']
    extra = ''
    if fb is not None:
        pos = fb.current_pose.pose.position
        extra = (f' final_pose=({pos.x:.2f}, {pos.y:.2f}) recoveries={fb.number_of_recoveries}'
                 f' nav_time={fb.navigation_time.sec}s')
    status = result.result().status
    print(f'RESULT {STATUS.get(status, status)}{extra} wall={time.time() - start:.1f}s')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if status == GoalStatus.STATUS_SUCCEEDED else 1


if __name__ == '__main__':
    sys.exit(main())
