# STATE — auto-trash-navigator

_last updated: 2026-09-14_

## Current Goal

**ローカルと `origin/main` を合流させる。** 残りは merge の実行判断のみ。
未コミットだった約3か月分の Windows 作業は
`feature/windows-cad-print-firmware` に退避済みで、もう失われない。

## Current Architecture

`origin/main` 側（= 実体）の構成:

```
src/                 現行 AMR ワークスペース（colcon）
  amr_bringup/        launch, world, patrol / pick_and_place / 検出器
  amr_description/    URDF/xacro（台車・アーム・センサ）
  amr_control/        コントローラ設定
  amr_moveit_config/  MoveIt 2（SRDF, kinematics, planning）
  amr_vision/         ビジョン
legacy/              旧 minicar ワークスペース（COLCON_IGNORE で除外）
6-arm-roboto/        実機アームの CAD  ← ローカルにあるのは主にここ
tools/ docs/ logs/ maps/
```

- ROS 2 Jazzy / Gazebo Harmonic (gz sim) / Nav2（AMCL + MPPI Omni）/ slam_toolbox / MoveIt 2
- トラッシュ検出は深度ベースの自己較正式（床平面フィッティング + 色フィルタ +
  既知位置ホワイトリストによる誤検出棄却）
- 開発機は Ubuntu 24.04。**この Windows のフォルダでは ROS 2 は動かない**

## Completed

git のタグが検証済みの節目を持っている（**すべて origin へ push 済み**）。
`CLAUDE.md` の表を参照。最新は `diff-drive-verified`（2026-09-02、差動駆動への変更 6/6）。

2026-07-24 の `project-complete-verified` で**無介入20分の統合検証に合格**している。
つまりこのプロダクトは「一度完成している」。その後 実機アーム → 実機シャーシ →
差動駆動 と、実機側へ寄せる改修が続いている。

**このセッションで自分が検証したものは無い**（ROS 2 が動く環境ではないため）。
上記はすべて git のタグとコミットメッセージから読んだ事実。

## Current Problems

1. **ブランチがまだ合流していない。** `feature/windows-cad-print-firmware`
   (9 コミット) は `main`(=`adc78e1`) の上にあり、`origin/main`(`db0d8db`) は
   そこから 95 コミット先。`git merge-tree` の dry-run では**衝突ゼロ**。
   実行するかはユーザー判断（§Blockers）。
2. **`ATN_print/A1_256/A1_S1_smallparts.stl` が古い。**
   生成 2026-08-31 18:54 に対し、入力の `camera_wedge_oak.stl` と
   `motor_bracket_*.stl` は 2026-09-03 14:17 更新。
   再生成すると 10996 → 12156 triangle に変わる。
   **このまま印刷すると古いカメラウェッジとモーターブラケットが出る。**
   保全のため現状のまま記録してあるので、印刷前に `python pack.py` を回すこと。
   （他の 9 プレートは再生成してもバイト単位で一致することを確認済み。）
3. ハンドブック（2026-07-02）の「現状サマリ」が 7 月時点のまま。
   §2 工程表・§4 検証コマンド・§6 リスク R1〜R12 は今でも有効。
4. ESP32 ファーム 3 世代のうち `atn_base_esp32/` は `CPR=3300` が仮定値のまま。
   実測済みは `_diff_v2` の `CPR=755`（7545/10回転）。世代の取捨選択が未整理。

## Assumptions

- ROS 2 開発は Ubuntu 24.04 機、この Windows は CAD・印刷・ファームの作業場。
  未コミットだった変更の内訳がその裏付け。
- `atn_base_esp32_diff_v2` が現行世代。`atn_base_esp32`(メカナム4輪独立) は
  差動駆動への移行で役目を終えている可能性が高いが、確認していない。

## Blockers

- **merge を実行するかがユーザー判断。**
  merge すると 95 コミット分の ROS 2 ワークスペース（`src/` `logs/` `maps/`
  `legacy/`）がこの Windows フォルダに実体化する。ここでは ROS 2 は動かない。
  「Windows にも全部置くか、CAD 専用のまま軽く保つか」はワークフローの好みで、
  技術的には決まらない。
- push も未実行（共有 remote を変えるため）。
- ROS 2 の検証はこの環境ではできない（Ubuntu 機が必要）。

## Next Best Actions

1. **合流方針を決めて実行する。** 衝突ゼロは確認済み。

   ```bash
   git checkout main
   git merge feature/windows-cad-print-firmware
   git merge origin/main
   git push origin main
   ```

   Windows を軽く保ちたい場合は、ブランチを push して統合は Ubuntu 側で行う。

   ```bash
   git push -u origin feature/windows-cad-print-firmware
   ```

2. 合流後、Ubuntu 側で CSV の改行を一度だけ正規化する
   （`docs/MULTI_OS_DEVELOPMENT.md` §4 にコマンドあり）。
3. `python pack.py` を回して `A1_S1_smallparts.stl` を最新化する（印刷前に必須）。
4. ESP32 ファームの `HALF_SEP=0.300` と、Ubuntu 側 `3ba9d0a` の
   `wheel_separation` が一致しているか実コードで突き合わせる。
   両者は独立に同じ結論へ達しているので、統合時の確認価値が高い。
5. ハンドブック §1 現状サマリを `diff-drive-verified` 時点へ更新する。
