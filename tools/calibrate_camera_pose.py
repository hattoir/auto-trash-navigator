#!/usr/bin/env python3
"""実機カメラのh(設置高さ)・θ(pitch)を校正するツール。

【重要】URDFの公称値(camera_height=0.20, camera_pitch_deg=15.0)を
信じてはいけない。実機の取り付け誤差・たわみ・ネジの締め具合などで
容易に数度・数cmずれる。mono_ranging.py の測距精度はθに極めて敏感
(2m先で1°ずれると約30cmの誤差)なので、実機組み立て後は必ず本ツールで
校正すること。

【使い方】
床に既知の距離(例: 0.5m, 1.0m, 1.5m)にマーカー(紙くずでも可)を置き、
そのマーカーが画像上のどこに写っているか(接地点の画素座標 u,v)と、
その真の距離(カメラから見た前方距離X、必要なら横方向Yも)を記録する。
u,vは実際の画像を目視 or yolo_trash_detector.pyのBBox下辺中心と
同じ考え方で読み取る(画像ビューアでピクセル座標を確認するのが簡単)。

観測点をJSON(下記フォーマット)で用意し、以下のように実行する:

  python3 tools/calibrate_camera_pose.py --observations obs.json \
      --fx 500 --fy 500 --cx 320 --cy 240

obs.json の例(u,v=接地点画素座標、x,y=真の床面座標[m]、yはカメラ正面
なら0でよい):
  [
    {"u": 320, "v": 300, "x": 0.5, "y": 0.0},
    {"u": 320, "v": 210, "x": 1.0, "y": 0.0},
    {"u": 320, "v": 160, "x": 2.0, "y": 0.0}
  ]

観測点は最低2点(h, θの2変数を決めるため)、できれば3点以上を
異なる距離で用意すること。

このツールはscipyに依存せず(実機環境でscipyがnumpy2との非互換で
壊れているケースが確認されているため)、粗い格子探索+段階的な
絞り込みで最小二乗フィットを行う。
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'amr_bringup', 'scripts'))
from mono_ranging import project_ground_point_to_pixel  # noqa: E402


def predict_ground_xy_from_pixel(u, v, fx, fy, cx, cy, h, theta_rad):
    """観測画素(u,v)を、候補(h,theta)で解釈した場合の床面(X,Y)を返す。
    mono_ranging.ground_plane_range()と同じ式をここでも使う
    (calibrate_camera_pose.py単体で完結させるため、依存を増やさない
    範囲で式を再掲する)。"""
    y_prime = (v - cy) / fy
    x_prime = (u - cx) / fx
    sin_t, cos_t = math.sin(theta_rad), math.cos(theta_rad)
    denom = sin_t + y_prime * cos_t
    if denom <= 1e-6:
        return None
    depth = h / denom
    x_ground = depth * (cos_t - y_prime * sin_t)
    y_ground = -x_prime * depth
    return x_ground, y_ground


def sse_for_params(observations, fx, fy, cx, cy, h, theta_rad):
    total = 0.0
    for obs in observations:
        pred = predict_ground_xy_from_pixel(obs['u'], obs['v'], fx, fy, cx, cy, h, theta_rad)
        if pred is None:
            return float('inf')
        px, py = pred
        total += (px - obs['x']) ** 2 + (py - obs.get('y', 0.0)) ** 2
    return total


def grid_search_fit(observations, fx, fy, cx, cy,
                     h_range=(0.05, 0.40), theta_deg_range=(2.0, 40.0),
                     n_coarse=60, n_refine_rounds=4, refine_factor=8.0):
    """粗い格子探索→段階的に範囲を狭めて再探索、を繰り返す
    (scipyのleast_squaresを使わない軽量な代替実装)。"""
    best_h, best_theta_deg, best_sse = None, None, float('inf')
    h_lo, h_hi = h_range
    th_lo, th_hi = theta_deg_range

    for round_i in range(n_refine_rounds):
        h_candidates = [h_lo + (h_hi - h_lo) * i / (n_coarse - 1) for i in range(n_coarse)]
        th_candidates = [th_lo + (th_hi - th_lo) * i / (n_coarse - 1) for i in range(n_coarse)]
        round_best = (None, None, float('inf'))
        for h in h_candidates:
            for th_deg in th_candidates:
                sse = sse_for_params(observations, fx, fy, cx, cy, h, math.radians(th_deg))
                if sse < round_best[2]:
                    round_best = (h, th_deg, sse)
        best_h, best_theta_deg, best_sse = round_best
        # 次ラウンドは今回のベスト周辺に範囲を絞る
        h_span = (h_hi - h_lo) / refine_factor
        th_span = (th_hi - th_lo) / refine_factor
        h_lo, h_hi = best_h - h_span, best_h + h_span
        th_lo, th_hi = best_theta_deg - th_span, best_theta_deg + th_span

    return best_h, best_theta_deg, best_sse


def self_test():
    """合成データ(既知のh,thetaから順算した観測点)でフィットが正しく
    復元できることを確認する自己テスト。"""
    print("=== 自己テスト: 合成データでのフィット復元検証 ===")
    fx, fy, cx, cy = 500.0, 500.0, 320.0, 240.0
    true_h, true_theta_deg = 0.22, 17.3  # URDF公称値からわざとずらした「実機の実際値」を模擬
    true_theta_rad = math.radians(true_theta_deg)

    synth_points = [(0.4, 0.0), (0.7, 0.1), (1.0, -0.1), (1.5, 0.0), (2.0, 0.05)]
    observations = []
    for (x, y) in synth_points:
        u, v = project_ground_point_to_pixel(x, y, fx, fy, cx, cy, true_h, true_theta_rad)
        observations.append({"u": u, "v": v, "x": x, "y": y})

    fit_h, fit_theta_deg, sse = grid_search_fit(observations, fx, fy, cx, cy)

    err_h = abs(fit_h - true_h)
    err_theta = abs(fit_theta_deg - true_theta_deg)
    print(f"真値: h={true_h:.4f}m theta={true_theta_deg:.3f}deg")
    print(f"推定: h={fit_h:.4f}m theta={fit_theta_deg:.3f}deg (SSE={sse:.2e})")
    print(f"誤差: h={err_h:.5f}m theta={err_theta:.4f}deg")
    ok = err_h < 0.002 and err_theta < 0.2
    print(f"=== 自己テスト {'PASSED' if ok else 'FAILED'} ===\n")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--observations", help="観測点JSONファイル(u,v,x,[y])のリスト")
    ap.add_argument("--fx", type=float, default=500.0)
    ap.add_argument("--fy", type=float, default=500.0)
    ap.add_argument("--cx", type=float, default=320.0)
    ap.add_argument("--cy", type=float, default=240.0)
    ap.add_argument("--self-test", action="store_true", help="合成データでの自己テストのみ実行")
    args = ap.parse_args()

    if args.self_test or not args.observations:
        ok = self_test()
        if not args.observations:
            print("(--observations が指定されていないため自己テストのみ実行しました)")
            sys.exit(0 if ok else 1)

    with open(os.path.expanduser(args.observations)) as f:
        observations = json.load(f)
    if len(observations) < 2:
        print("観測点は最低2点必要です", file=sys.stderr)
        sys.exit(1)

    fit_h, fit_theta_deg, sse = grid_search_fit(observations, args.fx, args.fy, args.cx, args.cy)
    rmse = math.sqrt(sse / len(observations))
    print(f"観測点数: {len(observations)}")
    print(f"推定結果: camera_height={fit_h:.4f} m, camera_pitch_deg={fit_theta_deg:.3f} deg")
    print(f"当てはめ残差RMSE: {rmse:.4f} m")
    print()
    print("yolo_trash_detector.py のパラメータへの設定例:")
    print(f"  camera_height: {fit_h:.4f}")
    print(f"  camera_pitch_deg: {fit_theta_deg:.3f}")


if __name__ == "__main__":
    main()
