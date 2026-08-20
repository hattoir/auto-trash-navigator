#!/usr/bin/env python3
"""autolabel_assist.pyの出力(多数の候補矩形)から学習用データセットを
組み立てる。

【重要な注記】本来はここでlabelImg等による人間のレビュー・修正が
入るべきだが、本セッションでは自律ループ内で完結させる必要があり
人間のレビューを挟めなかった。そのため、簡易的な妥協策として
「各画像で最大面積の候補矩形を、その画像内の唯一の紙くずとみなす」
というヒューリスティックでラベルを1画像1矩形に絞り込んでいる。
これは人間によるレビューの代用にはならない粗い近似であり、
学習結果(mAP等)はこの前提の誤り(背景の別物を最大矩形として
誤って選んでしまうケースなど)を反映した「ラフなベースライン」
として扱うこと。本番運用前に必ずlabelImg等で作り直すこと。

train/valを8:2に分割し、YOLO形式のディレクトリ構成とdata.yamlを生成する。
"""
import argparse
import os
import random
import shutil


def read_yolo_label(path):
    boxes = []
    if not os.path.exists(path):
        return boxes
    with open(path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 5:
                continue
            cls, xc, yc, w, h = parts
            boxes.append((int(cls), float(xc), float(yc), float(w), float(h)))
    return boxes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True)
    ap.add_argument("--draft-labels", required=True, help="autolabel_assist.pyの出力ディレクトリ")
    ap.add_argument("--out", required=True, help="出力データセットのルートディレクトリ")
    ap.add_argument("--val-frac", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--keep-largest-only", action="store_true", default=True,
                     help="1画像につき最大面積の矩形のみ残す(人間レビュー未実施の代替措置)")
    args = ap.parse_args()

    images_dir = os.path.expanduser(args.images)
    labels_dir = os.path.expanduser(args.draft_labels)
    out_dir = os.path.expanduser(args.out)

    exts = (".jpg", ".jpeg", ".png")
    files = sorted(f for f in os.listdir(images_dir) if f.lower().endswith(exts))
    random.Random(args.seed).shuffle(files)

    n_val = max(1, int(len(files) * args.val_frac))
    val_files = set(files[:n_val])
    train_files = [f for f in files if f not in val_files]

    for split, split_files in (("train", train_files), ("val", sorted(val_files))):
        img_out = os.path.join(out_dir, "images", split)
        lbl_out = os.path.join(out_dir, "labels", split)
        os.makedirs(img_out, exist_ok=True)
        os.makedirs(lbl_out, exist_ok=True)
        empty_count = 0
        for fname in split_files:
            stem = os.path.splitext(fname)[0]
            src_img = os.path.join(images_dir, fname)
            src_lbl = os.path.join(labels_dir, stem + ".txt")
            shutil.copy2(src_img, os.path.join(img_out, fname))

            boxes = read_yolo_label(src_lbl)
            if args.keep_largest_only and boxes:
                boxes = [max(boxes, key=lambda b: b[3] * b[4])]
            if not boxes:
                empty_count += 1
            with open(os.path.join(lbl_out, stem + ".txt"), "w") as f:
                for (cls, xc, yc, w, h) in boxes:
                    f.write(f"{cls} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}\n")
        print(f"{split}: {len(split_files)} images, {empty_count} with no label")

    data_yaml = os.path.join(out_dir, "data.yaml")
    with open(data_yaml, "w") as f:
        f.write(f"path: {out_dir}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write("names:\n")
        f.write("  0: trash\n")
    print(f"data.yaml written to {data_yaml}")


if __name__ == "__main__":
    main()
