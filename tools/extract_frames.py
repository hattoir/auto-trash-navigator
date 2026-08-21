#!/usr/bin/env python3
"""動画から等間隔でフレームを抽出し、実機カメラの見た目(640x480,
横長4:3)に合わせて中央クロップ+リサイズする。

抽出レートの決め方:
  1. ~/trash_dataset/videos/ 内の全動画の合計秒数を求める
  2. 目標総枚数(デフォルト325枚、指示の250〜400枚の中央値)を
     合計秒数で割り、「1秒あたり何枚抽出するか」のレートを逆算する
     (=2枚/秒を機械的に固定すると合計秒数次第で250〜400枚の範囲を
     外れることがあるため、実際の動画長から自動計算する)
  3. 各動画には長さに比例した枚数を割り当てる(このレートで等間隔抽出)
  4. 直前に採用したフレームとのグレースケールヒストグラム差分
     (相関係数)が閾値以上(=似すぎている)ならスキップする重複排除を
     適用する。そのため最終的な出力枚数は上記の目標より少なくなる
     (これは意図した動作)。

前処理(クロップ/リサイズ)について:
  入力動画のアスペクト比が目標(640x480 = 4:3)と異なる場合は、
  中央を基準にオーバーサイズな方の軸をクロップしてアスペクト比を
  4:3に合わせてから640x480へリサイズする。アスペクト比が既に4:3の
  場合はクロップせず単純リサイズする。どちらの処理を行ったかを
  動画ごとに報告する。
"""
import argparse
import os

import cv2
import numpy as np

TARGET_W, TARGET_H = 640, 480
TARGET_ASPECT = TARGET_W / TARGET_H


def crop_and_resize(frame, target_w=TARGET_W, target_h=TARGET_H):
    h, w = frame.shape[:2]
    src_aspect = w / h
    did_crop = False
    if abs(src_aspect - TARGET_ASPECT) > 1e-3:
        did_crop = True
        if src_aspect > TARGET_ASPECT:
            # 横長すぎる -> 左右をクロップ
            new_w = int(round(h * TARGET_ASPECT))
            x0 = (w - new_w) // 2
            frame = frame[:, x0:x0 + new_w]
        else:
            # 縦長すぎる -> 上下をクロップ
            new_h = int(round(w / TARGET_ASPECT))
            y0 = (h - new_h) // 2
            frame = frame[y0:y0 + new_h, :]
    resized = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)
    return resized, did_crop


def hist_similarity(img_a, img_b):
    """グレースケールヒストグラムの相関係数(1.0=完全一致)を返す。"""
    ga = cv2.cvtColor(img_a, cv2.COLOR_BGR2GRAY)
    gb = cv2.cvtColor(img_b, cv2.COLOR_BGR2GRAY)
    ha = cv2.calcHist([ga], [0], None, [64], [0, 256])
    hb = cv2.calcHist([gb], [0], None, [64], [0, 256])
    cv2.normalize(ha, ha)
    cv2.normalize(hb, hb)
    return cv2.compareHist(ha, hb, cv2.HISTCMP_CORREL)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--videos", default="~/trash_dataset/videos")
    ap.add_argument("--out", default="~/trash_dataset/frames")
    ap.add_argument("--target-total", type=int, default=325,
                     help="重複排除前の目標総抽出枚数(250-400の中央値がデフォルト)")
    ap.add_argument("--dedup-thresh", type=float, default=0.985,
                     help="ヒストグラム相関係数がこれ以上なら直前フレームと"
                          "似すぎているとみなしスキップ")
    args = ap.parse_args()

    videos_dir = os.path.expanduser(args.videos)
    out_dir = os.path.expanduser(args.out)
    os.makedirs(out_dir, exist_ok=True)

    exts = (".mov", ".mp4")
    files = sorted(f for f in os.listdir(videos_dir) if f.lower().endswith(exts))
    if not files:
        print(f"No video files found in {videos_dir}")
        return

    # --- ステップ1: 合計秒数を求める ---
    video_info = []
    total_duration = 0.0
    for fname in files:
        path = os.path.join(videos_dir, fname)
        cap = cv2.VideoCapture(path)
        fps = cap.get(cv2.CAP_PROP_FPS)
        n_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        duration = n_frames / fps if fps > 0 else 0.0
        video_info.append({'fname': fname, 'fps': fps, 'n_frames': n_frames,
                            'w': w, 'h': h, 'duration': duration})
        total_duration += duration
        cap.release()

    # --- ステップ2: レートを逆算 ---
    rate_per_sec = args.target_total / total_duration if total_duration > 0 else 2.0
    print(f"=== 抽出レートの計算過程 ===")
    print(f"動画本数: {len(files)}, 合計秒数: {total_duration:.1f}s")
    print(f"目標総枚数(重複排除前): {args.target_total}")
    print(f"逆算レート: {args.target_total} / {total_duration:.1f}s = {rate_per_sec:.3f} 枚/秒")
    print(f"(目安の2枚/秒からの調整: {rate_per_sec:.3f}/2.0 = {rate_per_sec/2.0:.3f}倍)")
    print()

    total_extracted = 0
    total_skipped_dup = 0
    seq = 0
    per_video_report = []

    for info in video_info:
        fname = info['fname']
        path = os.path.join(videos_dir, fname)
        cap = cv2.VideoCapture(path)
        fps = info['fps']
        step_frames = max(1, int(round(fps / rate_per_sec)))

        stem = os.path.splitext(fname)[0]
        n_this_video = 0
        n_skipped_this_video = 0
        last_kept_frame = None
        did_crop_reported = None

        frame_idx = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            if frame_idx % step_frames == 0:
                resized, did_crop = crop_and_resize(frame)
                if did_crop_reported is None:
                    did_crop_reported = did_crop
                if last_kept_frame is not None:
                    sim = hist_similarity(resized, last_kept_frame)
                    if sim >= args.dedup_thresh:
                        n_skipped_this_video += 1
                        frame_idx += 1
                        continue
                out_name = f"{stem}_{seq:05d}.jpg"
                cv2.imwrite(os.path.join(out_dir, out_name), resized,
                            [cv2.IMWRITE_JPEG_QUALITY, 92])
                last_kept_frame = resized
                seq += 1
                n_this_video += 1
            frame_idx += 1
        cap.release()

        total_extracted += n_this_video
        total_skipped_dup += n_skipped_this_video
        per_video_report.append({
            'fname': fname, 'duration': info['duration'],
            'src_res': f"{info['w']}x{info['h']}",
            'extracted': n_this_video, 'skipped_dup': n_skipped_this_video,
            'did_crop': did_crop_reported,
        })

    print("=== 動画ごとの抽出結果 ===")
    for r in per_video_report:
        crop_desc = "中央クロップ+リサイズ" if r['did_crop'] else "リサイズのみ(クロップ不要)"
        print(f"{r['fname']}: 元解像度={r['src_res']}, 長さ={r['duration']:.1f}s, "
              f"抽出={r['extracted']}枚, 重複排除でスキップ={r['skipped_dup']}枚, "
              f"前処理={crop_desc}")

    print()
    print(f"=== 合計 ===")
    print(f"総抽出枚数(重複排除後): {total_extracted}")
    print(f"重複排除でスキップした枚数: {total_skipped_dup}")
    print(f"出力解像度: {TARGET_W}x{TARGET_H}")
    print(f"出力先: {out_dir}")


if __name__ == "__main__":
    main()
