# STATE — auto-trash-navigator

_last updated: 2026-09-14_

## Current Goal

**コードを書く前に、作業ツリーの状態をユーザーと揃える。**
ローカル `main` が `origin/main` から 89 コミット遅れており、
この状態で ROS 2 側に手を入れるとほぼ確実に事故になる。

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

1. **ローカル `main` が 3 か月古い**（adc78e1 / 2026-06-16 対 origin/main db0d8db / 2026-09-02）。
   このフォルダだけを見ると「CAD ファイルしか無いプロジェクト」に見えてしまう。
   実害: このリポジトリを読んだ人間や AI が、プロダクトの規模と現在地を誤認する。
2. **ローカル `README.md` が 1 行しか無い**（しかも `# aout-trash-navigator` と綴りが誤っている）。
   ただし `origin/main` の README は充実しているので、**ローカルで README を書き直してはいけない**。
   古いコミットの上に新しい README を作ると、追いついたときに無駄な衝突になる。
   → 直し方は「ローカルを追いつかせる」であって「README を書く」ではない。
3. ハンドブック（2026-07-02）の「現状サマリ」が 7 月時点のままで、実際の進捗より遅れている。
   ただし §2 工程表・§4 検証コマンド・§6 リスク R1〜R12 は今でも有効。
4. `6-arm-roboto` 配下に `.step` と `.f3d` がそのまま入っている（バイナリ）。
   git が肥大する。今は実害が小さいので放置しているが、増え続けるなら要検討。

## Assumptions

- ROS 2 の開発は別マシン（Ubuntu 24.04）で行われ、この Windows フォルダは
  主に CAD と紙物（印刷・手順・測定シート）の作業場である、と理解した。
  未コミットの変更内容（`.step` 12件 + `ATN_print/` + 手順書・測定シート）がその裏付け。
- したがってローカルを origin へ追いつかせることの優先度は、
  「ここで ROS 2 を触るかどうか」に依存する。**ユーザーに聞くべきこと。**

## Blockers

- **ローカルに未コミットのユーザー変更がある。**
  `6-arm-roboto/3Dmodel/*.step` 12 件が変更、`Base_J1.f3d` `CameraMount.step`
  `CameraRetainer.step` `GripperTop.step` `J2_Bracket_v3.step` が新規、
  `ATN_day_procedure.md` `ATN_measurement_sheet.md` `ATN_print/` が新規。
  **`git pull` / `git checkout` / `git reset` を自律実行していない。**
  ユーザーの作業中の CAD を失う可能性があるため、OS §2・§23 により止めてある。
  → ユーザー判断が必要。手順の案:
    1. まず未コミット分を意味のあるコミットにする（CAD の変更 + 新しい手順書）
    2. その上で `git pull`（origin/main は 89 コミット先なので merge か rebase を選ぶ）
    3. `.step` の衝突が出たらユーザーが正しい版を選ぶ（機械的には解決できない）
- ROS 2 の検証はこの環境ではできない（Ubuntu 機が必要）。

## Next Best Actions

1. **ユーザーに確認する**（この 1 つだけが今できる有益な作業）:
   未コミットの CAD 変更をコミットしてから origin/main に追いつくか、
   それともこの Windows 側は CAD 専用として古い main のままで良いか。
   後者なら、そう `DECISIONS.md` に記録して以後迷わないようにする。
2. 追いつく方針が決まったら、その手順を実行する（上の Blockers の 3 手順）。
3. 追いついた後にやる価値があること: ハンドブックの「§1 現状サマリ」を
   現在の到達点（`diff-drive-verified`）に合わせて更新する。
   このドキュメントは新しいセッションに貼る前提で作られているので、
   ここが古いと毎回誤ったコンテキストを配ることになる。
4. `README.md` のローカル 1 行版は、追いついた時点で自動的に解決する（何もしない）。
