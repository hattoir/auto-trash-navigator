#!/usr/bin/env python3
"""Phase 2-D: 3点Waypoint無限巡回 (nav2_simple_commander)

使い方:
  スポーン直後(0,0にいる)なら:
    ros2 run amr_bringup patrol.py --ros-args -p set_initial_pose:=true
  すでにRVizの2D Pose EstimateでAMCLを初期化済みなら:
    ros2 run amr_bringup patrol.py

Waypoint選定根拠:
  壁(±4m)から1.6m、障害物中心(2,2)(-2,-2)(-2,2)から2.0m以上離れた
  3点で部屋を大きく三角形に周回する。
    A( 2.4, -2.4) : 障害物のない南東コーナー
    B( 0.0,  2.4) : 北辺中央(障害物(2,2)(-2,2)の間)
    C(-2.4,  0.0) : 西辺中央(障害物(-2,2)(-2,-2)の間)
各Waypointでは次のWaypoint方向を向く(カメラ前方=進行方向を維持、Phase 3準備)。
"""
import math
import time

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

WAYPOINTS = [
    (2.4, -2.4),
    (0.0, 2.4),
    (-2.4, 0.0),
]
MAX_CONSECUTIVE_FAILURES = 3
GOAL_TIMEOUT_SEC = 120.0


def make_pose(navigator, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = navigator.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.declare_parameter('set_initial_pose', False)

    if navigator.get_parameter('set_initial_pose').value:
        navigator.get_logger().info('初期位置 (0,0,0) を設定します')
        navigator.setInitialPose(make_pose(navigator, 0.0, 0.0, 0.0))

    navigator.get_logger().info('Nav2 の active 化を待機中...')
    navigator.waitUntilNav2Active(localizer='amcl')
    navigator.get_logger().info('Nav2 active。巡回を開始します')

    lap = 0
    consecutive_failures = 0
    try:
        while rclpy.ok():
            lap += 1
            lap_start = time.monotonic()
            for i, (x, y) in enumerate(WAYPOINTS):
                # 次のWaypoint方向を向くyawを計算
                nx, ny = WAYPOINTS[(i + 1) % len(WAYPOINTS)]
                yaw = math.atan2(ny - y, nx - x)
                label = f'Lap{lap} WP{i + 1}/{len(WAYPOINTS)} ({x:+.1f},{y:+.1f})'
                navigator.get_logger().info(f'{label} へ移動開始')

                navigator.goToPose(make_pose(navigator, x, y, yaw))
                nav_start = navigator.get_clock().now()
                while not navigator.isTaskComplete():
                    feedback = navigator.getFeedback()
                    if feedback and (navigator.get_clock().now() - nav_start
                                     > Duration(seconds=GOAL_TIMEOUT_SEC)):
                        navigator.get_logger().warn(f'{label} タイムアウト。キャンセルします')
                        navigator.cancelTask()
                    time.sleep(0.2)

                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    navigator.get_logger().info(f'{label} 到達')
                    consecutive_failures = 0
                else:
                    consecutive_failures += 1
                    navigator.get_logger().warn(
                        f'{label} 失敗 (result={result}, 連続{consecutive_failures}回)。'
                        'スキップして次へ')
                    if consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                        navigator.get_logger().error(
                            f'{MAX_CONSECUTIVE_FAILURES}回連続で失敗。巡回を中断します')
                        return
            navigator.get_logger().info(
                f'=== Lap{lap} 完了: {time.monotonic() - lap_start:.1f} 秒 ===')
    except KeyboardInterrupt:
        navigator.get_logger().info('Ctrl+C: タスクをキャンセルして終了します')
        navigator.cancelTask()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
