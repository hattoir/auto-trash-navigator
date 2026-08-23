#!/usr/bin/env python3
"""mono_ranging.py の単体テスト。

1. 往復検証: 既知の床面上の距離(0.3/0.5/1.0/2.0m)にある点の画像座標を
   順算(project_ground_point_to_pixel)し、それを逆算
   (ground_plane_range)して元の距離が復元できるか(往復誤差<1mm)を確認する。
2. 感度解析: h(カメラ高さ)を±1cm、θ(pitch)を±1°変えたときに、
   実際の距離推定がどれだけずれるかを表で出力する。理論上、θの誤差は
   距離の2乗に近い形で効いてくる(遠いほど誤差が拡大する)ことを
   数値で確認する。
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'amr_bringup', 'scripts'))
from mono_ranging import ground_plane_range, project_ground_point_to_pixel  # noqa: E402

# テストで使うカメラ内部パラメータ(典型的な640x480相当を想定)
FX, FY = 500.0, 500.0
CX, CY = 320.0, 240.0

CAMERA_HEIGHT = 0.20   # m
CAMERA_PITCH_DEG = 15.0
CAMERA_PITCH_RAD = math.radians(CAMERA_PITCH_DEG)


def optical_to_ground(x_opt, y_opt, z_opt, pitch_rad, h):
    """光学フレームの3D点を床面座標(前方X, 横Y)に変換する(テスト専用の
    検証ヘルパー。depth_trash_detector.pyのXf/Yl変換と同じ式)。
    Zはh - (sinθ*z_opt + cosθ*y_opt)で計算し、0に近いことも確認に使う。"""
    sin_t, cos_t = math.sin(pitch_rad), math.cos(pitch_rad)
    x_ground = cos_t * z_opt - sin_t * y_opt
    y_ground = -x_opt
    z_ground = h - (sin_t * z_opt + cos_t * y_opt)
    return x_ground, y_ground, z_ground


def round_trip_test():
    print("=== 往復検証(h=0.20m, theta=15deg) ===")
    test_distances = [0.3, 0.5, 1.0, 2.0]
    max_err = 0.0
    all_ok = True
    for true_x in test_distances:
        for true_y in [0.0, 0.15, -0.15]:
            u, v = project_ground_point_to_pixel(
                true_x, true_y, FX, FY, CX, CY, CAMERA_HEIGHT, CAMERA_PITCH_RAD)
            result = ground_plane_range(u, v, FX, FY, CX, CY, CAMERA_HEIGHT, CAMERA_PITCH_RAD)
            if result is None:
                print(f"  X={true_x}, Y={true_y}: 逆算失敗(Noneが返った)")
                all_ok = False
                continue
            rec_x, rec_y, rec_z = optical_to_ground(
                result.x_opt, result.y_opt, result.z_opt, CAMERA_PITCH_RAD, CAMERA_HEIGHT)
            err_x = abs(rec_x - true_x)
            err_y = abs(rec_y - true_y)
            err_z = abs(rec_z - 0.0)
            max_err = max(max_err, err_x, err_y, err_z)
            ok = err_x < 1e-3 and err_y < 1e-3 and err_z < 1e-3
            all_ok &= ok
            print(f"  真値X={true_x:.3f} Y={true_y:.3f} -> pixel=({u:.2f},{v:.2f}) -> "
                  f"復元X={rec_x:.6f} Y={rec_y:.6f} Z={rec_z:.6f} "
                  f"誤差(X,Y,Z)=({err_x:.2e},{err_y:.2e},{err_z:.2e}) {'OK' if ok else 'NG'}")
    print(f"\n最大誤差: {max_err:.2e} m ({'< 1mmで合格' if max_err < 1e-3 else '不合格'})")
    print(f"=== 往復検証 {'PASSED' if all_ok and max_err < 1e-3 else 'FAILED'} ===\n")
    return all_ok and max_err < 1e-3


def sensitivity_table():
    print("=== 感度解析: h(±1cm) / theta(±1deg) の誤差が距離推定に与える影響 ===")
    distances = [0.3, 0.5, 1.0, 1.5, 2.0, 2.5]
    dh = 0.01       # 1cm
    dtheta = math.radians(1.0)  # 1deg

    header = f"{'X_true[m]':>10} | {'h+1cm誤差[m]':>14} | {'h-1cm誤差[m]':>14} | {'th+1deg誤差[m]':>15} | {'th-1deg誤差[m]':>15}"
    print(header)
    print("-" * len(header))

    rows = []
    for true_x in distances:
        # 真の(h, theta)で対象を観測した時の画像座標(観測は変わらない、
        # 変わるのは「その画像座標をどのh,thetaで解釈するか」)
        u, v = project_ground_point_to_pixel(
            true_x, 0.0, FX, FY, CX, CY, CAMERA_HEIGHT, CAMERA_PITCH_RAD)

        def estimate_x(h, theta):
            r = ground_plane_range(u, v, FX, FY, CX, CY, h, theta)
            if r is None:
                return None
            x_g, _, _ = optical_to_ground(r.x_opt, r.y_opt, r.z_opt, theta, h)
            return x_g

        est_h_plus = estimate_x(CAMERA_HEIGHT + dh, CAMERA_PITCH_RAD)
        est_h_minus = estimate_x(CAMERA_HEIGHT - dh, CAMERA_PITCH_RAD)
        est_th_plus = estimate_x(CAMERA_HEIGHT, CAMERA_PITCH_RAD + dtheta)
        est_th_minus = estimate_x(CAMERA_HEIGHT, CAMERA_PITCH_RAD - dtheta)

        err_h_plus = (est_h_plus - true_x) if est_h_plus is not None else float('nan')
        err_h_minus = (est_h_minus - true_x) if est_h_minus is not None else float('nan')
        err_th_plus = (est_th_plus - true_x) if est_th_plus is not None else float('nan')
        err_th_minus = (est_th_minus - true_x) if est_th_minus is not None else float('nan')

        rows.append((true_x, err_h_plus, err_h_minus, err_th_plus, err_th_minus))
        print(f"{true_x:>10.2f} | {err_h_plus:>+14.4f} | {err_h_minus:>+14.4f} | "
              f"{err_th_plus:>+15.4f} | {err_th_minus:>+15.4f}")

    print()
    # 2m地点でtheta誤差1degがどれだけずれるか、ハイライトして報告
    for row in rows:
        if abs(row[0] - 2.0) < 1e-6:
            print(f"確認: 2.0m地点でtheta+1deg誤差 = {row[3]:+.4f}m "
                  f"(theta-1deg誤差 = {row[4]:+.4f}m)")
    return rows


if __name__ == "__main__":
    ok = round_trip_test()
    sensitivity_table()
    if not ok:
        sys.exit(1)
