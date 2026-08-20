#!/usr/bin/env python3
"""アノテーション補助スクリプト(たたき台生成専用)。

【重要】このスクリプトが出力するラベルは人間による確認・修正が
前提の「たたき台」であり、学習にそのまま使ってよい正解データでは
ない。明度/彩度/形状のヒューリスティックで「紙くずらしいブロブ」の
候補矩形を抽出しているだけで、背景の白い物体(壁、床の照明反射、
紙以外の白い物)も候補として拾ってしまう。必ず labelImg 等の
アノテーションツールで人間が目視確認し、誤検出の削除・見逃しの
追加・矩形の微調整を行ってから学習に使うこと。

想定するヒューリスティック:
  - 紙くず(白色・薄い色の紙)は背景(床/机など)に対して
    「明るい(高いV)」「彩度が低い(低いS)」領域として現れやすい。
  - 紙くずはある程度まとまった塊(小さすぎるノイズや、画像全体を
    覆うような大きすぎる領域は除外)。
  - 極端に細長い領域(照明の反射・境界線などが多い)は除外。

使い方:
  python3 tools/autolabel_assist.py \
      --images ~/trash_dataset/images_jpg \
      --labels ~/trash_dataset/labels_draft \
      [--class-id 0] [--min-area-frac 0.0008] [--max-area-frac 0.25]
"""
import argparse
import os
import sys

import cv2
import numpy as np


def find_candidate_boxes(img_bgr, min_area_frac, max_area_frac,
                          v_thresh=170, s_thresh=80, max_aspect=6.0,
                          edge_density_thresh=0.03):
    h, w = img_bgr.shape[:2]
    img_area = h * w

    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    s_channel = hsv[:, :, 1]
    v_channel = hsv[:, :, 2]

    # 明るく(高V)、彩度が低い(低S) = 白っぽい紙くずらしい領域
    bright_mask = ((v_channel >= v_thresh) & (s_channel <= s_thresh)).astype(np.uint8) * 255

    # くしゃくしゃに丸まった紙は皺による局所的なエッジ(陰影)が多い一方、
    # 白い机/壁のような平坦な背景はエッジがほとんどない。単純な色閾値
    # だけだと紙くずが背景の白い平面と同じ塊に融合してしまい
    # (max_area_fracを超えて丸ごと除外される)、肝心の対象を見逃す
    # ケースがあったため、エッジ密度でも絞り込む。
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 40, 120)
    edge_density = cv2.boxFilter((edges > 0).astype(np.float32), -1, (41, 41))
    textured_mask = (edge_density >= edge_density_thresh).astype(np.uint8) * 255

    mask = cv2.bitwise_and(bright_mask, textured_mask)

    # ノイズ除去(小さい穴を埋め、孤立した小さいノイズを消す)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    boxes = []
    for c in contours:
        area = cv2.contourArea(c)
        area_frac = area / img_area
        if area_frac < min_area_frac or area_frac > max_area_frac:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        aspect = max(bw, bh) / max(1, min(bw, bh))
        if aspect > max_aspect:
            continue
        boxes.append((x, y, bw, bh, area_frac))
    return boxes, mask


def to_yolo_line(class_id, x, y, bw, bh, img_w, img_h):
    xc = (x + bw / 2.0) / img_w
    yc = (y + bh / 2.0) / img_h
    nw = bw / img_w
    nh = bh / img_h
    return f"{class_id} {xc:.6f} {yc:.6f} {nw:.6f} {nh:.6f}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--images", required=True, help="入力画像ディレクトリ(jpg/png)")
    ap.add_argument("--labels", required=True, help="出力ラベルディレクトリ(YOLO形式 .txt)")
    ap.add_argument("--class-id", type=int, default=0)
    ap.add_argument("--min-area-frac", type=float, default=0.0008,
                     help="画像全体に対する最小面積比(これより小さい候補は除外)")
    ap.add_argument("--max-area-frac", type=float, default=0.25,
                     help="画像全体に対する最大面積比(これより大きい候補は除外、背景誤検出対策)")
    ap.add_argument("--v-thresh", type=int, default=170, help="HSVのV(明度)閾値、これ以上を候補に")
    ap.add_argument("--s-thresh", type=int, default=80, help="HSVのS(彩度)閾値、これ以下を候補に")
    ap.add_argument("--max-aspect", type=float, default=6.0, help="縦横比の上限(これを超える細長い領域は除外)")
    ap.add_argument("--edge-density-thresh", type=float, default=0.03,
                     help="局所エッジ密度の下限(これ未満の平坦領域=背景とみなし除外)")
    ap.add_argument("--debug-vis", default=None,
                     help="指定すると、候補矩形を描画した確認用画像をこのディレクトリに保存する")
    args = ap.parse_args()

    images_dir = os.path.expanduser(args.images)
    labels_dir = os.path.expanduser(args.labels)
    os.makedirs(labels_dir, exist_ok=True)
    if args.debug_vis:
        os.makedirs(os.path.expanduser(args.debug_vis), exist_ok=True)

    exts = (".jpg", ".jpeg", ".png")
    files = sorted(f for f in os.listdir(images_dir) if f.lower().endswith(exts))
    if not files:
        print(f"No images found in {images_dir}", file=sys.stderr)
        sys.exit(1)

    total_boxes = 0
    empty_count = 0
    for fname in files:
        img_path = os.path.join(images_dir, fname)
        img = cv2.imread(img_path)
        if img is None:
            print(f"  skip (failed to read): {fname}", file=sys.stderr)
            continue
        h, w = img.shape[:2]

        boxes, mask = find_candidate_boxes(
            img, args.min_area_frac, args.max_area_frac,
            v_thresh=args.v_thresh, s_thresh=args.s_thresh, max_aspect=args.max_aspect,
            edge_density_thresh=args.edge_density_thresh)

        stem = os.path.splitext(fname)[0]
        label_path = os.path.join(labels_dir, stem + ".txt")
        with open(label_path, "w") as f:
            for (x, y, bw, bh, _area_frac) in boxes:
                f.write(to_yolo_line(args.class_id, x, y, bw, bh, w, h) + "\n")

        total_boxes += len(boxes)
        if not boxes:
            empty_count += 1

        if args.debug_vis:
            vis = img.copy()
            for (x, y, bw, bh, area_frac) in boxes:
                cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 3)
                cv2.putText(vis, f"{area_frac:.4f}", (x, max(0, y - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imwrite(os.path.join(os.path.expanduser(args.debug_vis), fname), vis)

    print(f"processed {len(files)} images")
    print(f"total candidate boxes: {total_boxes} (avg {total_boxes / max(1, len(files)):.2f} per image)")
    print(f"images with zero candidates: {empty_count}")
    print(f"labels written to: {labels_dir}")
    print()
    print("*** これはたたき台です。labelImg 等で必ず人間が確認・修正してください。 ***")


if __name__ == "__main__":
    main()
