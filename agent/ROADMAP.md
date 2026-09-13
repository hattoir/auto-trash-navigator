# ROADMAP — auto-trash-navigator

## ⚠ 正本は開発ロードマップ・ハンドブック

`Auto-Trash-Navigator_開発ロードマップ_Gemini移行ハンドブック.md`（2026-07-02 作成）が
Phase 2〜6 の工程表を持っている。**ここで二重に持たない。**

- 工程表（Phase 2-A 〜 Phase 6） → ハンドブック §2
- 検証コマンド → ハンドブック §4
- 既知の罠 R1〜R12 → ハンドブック §6
- 進め方の推奨順序 → ハンドブック §7

**ただしハンドブックの §1「プロジェクト現状サマリ」は 7 月時点のもので古い。**
実際は `project-complete-verified`（2026-07-24、無介入20分の統合検証合格）を越えて
`diff-drive-verified`（2026-09-02）まで進んでいる。

## 実際に到達している地点（git のタグ）

| タグ | 日付 | 内容 |
|---|---|---|
| `phase1-complete` | 2026-07-03 | 台車の基盤（このPCの完動状態） |
| `phase2-complete` | 2026-07-07 | Nav2 巡回、base_link 座標崩れの修正 |
| `phase4-complete` | 2026-07-13 | ピック&プレース（10/10 中央・5/5 L・5/5 R） |
| `project-complete-verified` | 2026-07-24 | 固定ダストボックスへの投下。無介入20分合格 |
| `real-arm-verified` | 2026-08-11 | 実機アーム |
| `software-complete` | 2026-08-24 | 3回目回収後の2周以上を厳格検証 |
| `hw-v2-verified` | 2026-09-01 | 実機シャーシ形状 6/6 |
| `diff-drive-verified` | 2026-09-02 | 差動駆動への変更 6/6 |

つまりハンドブックの Phase 2〜5 は**完了している**。
残っているのはハンドブック §2 の **Phase 6（実機移行）**。
実際の動きもそちらへ向かっている（実機アーム → 実機シャーシ → 差動駆動）。

## いま (Now)

**ローカル作業ツリーを origin に追いつかせるかを決める。** それが済むまで ROS 2 側は触れない。
詳細は `agent/STATE.md` の Blockers。

## 次 (Next)

ハンドブック §2 Phase 6（実機移行）。メカナム → 差動駆動への変更が既に入っているので、
実機の駆動系に合わせる作業が進行中と読める。

## 作らないと決めたもの (Non-goals)

- **Gazebo Classic / ROS 1 の資産を使わない。** gz-sim 系プラグイン + `ros_gz_bridge` / `ros_gz_sim` が正解。
  `libgazebo_ros_*.so`・`spawn_entity.py`・`gazebo_ros` パッケージはすべて不正解（ハンドブック §1）。
- **`legacy/` の旧 minicar ワークスペースを復活させない**（`COLCON_IGNORE` で除外済み）。
