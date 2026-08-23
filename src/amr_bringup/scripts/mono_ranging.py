#!/usr/bin/env python3
"""単眼カメラ + 床面仮定による測距(monocular ground-plane ranging)。

実機カメラ(Raspberry Pi 5 + Piカメラ)には深度センサーがないため、
depth_trash_detector.py が使っている「深度画像から測距」は使えない。
代わりに「対象は床(z≈0)に接している」という仮定と、カメラの設置高さ h・
下向き角(pitch)θ が既知であるという前提を使い、画像上の1点から
床上の3D位置を一意に求める。

【原理】depth_trash_detector.py の床面自己較正で使っているのと同じ
ピンホールモデル+床面交差の式:
  y' = (v - cy) / fy   (画像中心からの正規化縦座標。光学フレームの
                         y=down規約に合わせ、下に行くほど正)
  x' = (u - cx) / fx   (正規化横座標)
  depth = h / (sinθ + y'・cosθ)   (カメラから対象までの光線距離)
  X_ground = depth・(cosθ − y'・sinθ)   (床面上の前方距離)
  Y_ground = −x'・depth                 (床面上の横方向)
  Z_ground = 0                          (床面上、定義より)

この関数はカメラ光学フレーム(x=right, y=down, z=forward)での
3D点 (x_opt, y_opt, z_opt) = (x'・depth, y'・depth, depth) を返す。
depth_trash_detector.py / yolo_trash_detector.py(depthモード)が
同じ光学フレーム規約で点をpublishしているため、そのままTF変換
(optical_frame -> base_footprint -> map、URDFのカメラ取り付け位置が
そのままh・θの回転・並進を担う)に載せられ、下流のmap変換・重複抑制・
境界棄却(trash_map_publish.py)を一切変更せずに再利用できる。
実際にX_ground/Y_ground/Z_groundの式に一致することは
tools/test_mono_ranging.pyの自己検証で確認している
(Zf = h - depth・(sinθ+y'cosθ) = h - h = 0 になることを利用)。

【精度の性格(重要、必ず理解した上で使うこと)】
この方式はθの角度誤差がそのまま距離誤差に直結する。感度解析
(tools/test_mono_ranging.pyのsensitivity_table())で示す通り、
例えば θ に 1° の誤差があると、2m先の対象では前方距離に
約30cmもの誤差が生じる(近距離ほど誤差は小さい: 0.3m先では
数mm〜1cm程度)。したがってこの測距は「近距離では実用的な精度、
遠距離では方向(だいたいどちらにあるか)の当たりをつける程度」
という性格のものとして扱うべきである。実運用では、遠方でざっくり
検出した対象へロボットが接近し、近距離で再検出することで精度を
上げていく前提の設計とすること(遠方の1回の検出結果だけを
最終的なpick位置として信用しない)。
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class GroundPlaneRangingResult:
    x_opt: float  # カメラ光学フレーム: right
    y_opt: float  # カメラ光学フレーム: down
    z_opt: float  # カメラ光学フレーム: forward (= depth, 光線距離)


def bbox_ground_contact_point(u1: float, v1: float, u2: float, v2: float) -> Tuple[float, float]:
    """BBox(u1,v1,u2,v2)から「接地点」の画素座標を返す。
    対象は床に接しているため、BBoxの下辺の中心を使う(BBox中心を使うと、
    物体の高さの分だけ実際より遠くにあると誤って推定してしまう)。"""
    u = (u1 + u2) / 2.0
    v = max(v1, v2)  # 画像座標は下に行くほどvが大きい前提(下辺 = v最大側)
    return u, v


def ground_plane_range(
    u: float, v: float,
    fx: float, fy: float, cx: float, cy: float,
    camera_height: float, camera_pitch_rad: float,
    min_denom: float = 1e-3,
) -> Optional[GroundPlaneRangingResult]:
    """画像座標(u,v)を床面仮定で3D化する。

    Args:
        u, v: 画素座標(接地点。bbox_ground_contact_point()の出力を渡す)
        fx, fy, cx, cy: camera_infoのピンホールモデル内部パラメータ
        camera_height: カメラの床からの設置高さ [m]
        camera_pitch_rad: カメラの下向き角 [rad](水平=0、真下=pi/2)
        min_denom: 分母(sinθ+y'cosθ)がこれ以下なら「地平線より上を
            向いている(床と交差しない)」とみなし測距不能としてNoneを返す

    Returns:
        GroundPlaneRangingResult、または測距不能ならNone
    """
    y_prime = (v - cy) / fy
    x_prime = (u - cx) / fx

    sin_t = math.sin(camera_pitch_rad)
    cos_t = math.cos(camera_pitch_rad)
    denom = sin_t + y_prime * cos_t

    if denom <= min_denom:
        return None

    depth = camera_height / denom
    if depth <= 0 or not math.isfinite(depth):
        return None

    x_opt = x_prime * depth
    y_opt = y_prime * depth
    z_opt = depth
    return GroundPlaneRangingResult(x_opt=x_opt, y_opt=y_opt, z_opt=z_opt)


def ground_plane_range_from_bbox(
    u1: float, v1: float, u2: float, v2: float,
    fx: float, fy: float, cx: float, cy: float,
    camera_height: float, camera_pitch_rad: float,
    min_denom: float = 1e-3,
) -> Optional[GroundPlaneRangingResult]:
    """BBoxから直接、床面仮定での3D位置を求める(接地点=下辺中心を使用)。"""
    u, v = bbox_ground_contact_point(u1, v1, u2, v2)
    return ground_plane_range(u, v, fx, fy, cx, cy, camera_height, camera_pitch_rad, min_denom)


def project_ground_point_to_pixel(
    x_ground: float, y_ground: float,
    fx: float, fy: float, cx: float, cy: float,
    camera_height: float, camera_pitch_rad: float,
) -> Tuple[float, float]:
    """順算: 床面上の点(前方x_ground, 横y_ground, z=0)を画像座標(u,v)に
    投影する。ground_plane_range()の逆演算であり、
    tools/test_mono_ranging.pyの往復検証・tools/calibrate_camera_pose.py
    の最小二乗フィットで使う。

    導出: depth_trash_detector.py と同じ回転(Xf=ct*zo-st*yo,
    Zf=h-(st*zo+ct*yo))の逆を解く。z=0(床面)の点について、
    光学フレームでの (xo,yo,zo) は:
      zo = X_ground*cosθ + h*sinθ  ... (a)
      yo = (h - X_ground*sinθ) / cosθ ... はcosθ=0で特異なため、
    連立を素直に解くと:
      zo = X_ground*cosθ + h*sinθ
      yo = h/cosθ - zo*tanθ  (Zf=0 より h - (st*zo+ct*yo)=0 を yo について解く)
      xo = -Y_ground
    """
    sin_t = math.sin(camera_pitch_rad)
    cos_t = math.cos(camera_pitch_rad)

    zo = x_ground * cos_t + camera_height * sin_t
    # Zf = h - (sin_t*zo + cos_t*yo) = 0  =>  yo = (h - sin_t*zo) / cos_t
    yo = (camera_height - sin_t * zo) / cos_t
    xo = -y_ground

    u = xo / zo * fx + cx if zo != 0 else float('nan')
    v = yo / zo * fy + cy if zo != 0 else float('nan')
    return u, v
