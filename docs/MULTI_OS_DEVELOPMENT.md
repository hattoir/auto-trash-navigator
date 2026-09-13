# MULTI-OS DEVELOPMENT — Ubuntu + Windows

_last updated: 2026-09-14_

Auto-Trash Navigator を **1 つのリポジトリのまま** Ubuntu と Windows の
両方から開発するための取り決め。OS ごとに別プロダクトへ分裂させないことが目的。

---

## 1. 役割分担

| | Ubuntu 24.04 (Primary) | Windows 11 (Secondary) |
|---|---|---|
| ROS 2 Jazzy / Gazebo Harmonic | **Primary** | 動かない |
| Nav2 / MoveIt 2 / slam_toolbox | **Primary** | 動かない |
| Vision (YOLO / depth) | **Primary** | 不可 |
| 実機ロボット runtime | **Primary** | 対象外 |
| Fusion 360 / CAD 編集 | 不可 | **Primary** |
| 3D print 組版 (`pack.py`) | 可 | **Primary** |
| ESP32 ファームウェア | 可 | **Primary**（Arduino IDE） |
| ドキュメント | 可 | 可 |

**Robot Runtime を Windows 必須にしない。** Windows は設計・造形・ファーム側。

## 2. Git が Source of Truth

フォルダを同期ソフトや USB でコピーしない。必ず remote 経由で往復させる。

```
Ubuntu ──commit──▶ origin ──pull──▶ Windows
Windows ──commit──▶ origin ──pull──▶ Ubuntu
```

`origin` = `git@github.com:hattoir/auto-trash-navigator.git`

## 3. 機械を切り替える前に

```bash
git status      # 未コミットを残さない
git add -A && git commit
git push
```

移った先で：

```bash
git fetch --all --prune
git pull
```

**これを怠ると何が起きるか**: 実際に 2026-06-16 〜 2026-09-14 の約 3 か月、
Windows 側が `adc78e1` に取り残されたまま CAD・印刷・ファームの作業が
未コミットで積み上がり、Ubuntu 側は 95 コミット先行していた。
どちらの成果も失われなかったが、片側からもう片側が見えない状態が続いた。

## 4. 改行コード / バイナリ

`.gitattributes` で明示している。方針：

- index は常に **LF**。working tree は OS ネイティブのままでよい。
- `.sh` は `eol=lf`、`.bat` / `.ps1` は `eol=crlf`（そうでないと壊れるため）。
- `.step` / `.stp` は ASCII なので **text**（index は LF）。
- `.stl` は **binary を明示**。理由：`pack.py` が出力する binary STL は
  先頭 80 byte が ASCII ヘッダ（`ATN ...`）なので、git の自動判定に
  頼るのは危険。`.3mf` `.f3d` も同様に binary 宣言。

### 既知の移行タスク（未実施）

`origin/main` 側の次の 2 ファイルは index に CRLF で入っている。

```
src/amr_bringup/scripts/height_scan.csv
src/amr_bringup/scripts/reach_map.csv
```

`.gitattributes` が合流したあと、Ubuntu 側で一度だけ正規化すること。

```bash
git add --renormalize src/amr_bringup/scripts/height_scan.csv src/amr_bringup/scripts/reach_map.csv
git commit -m "chore: renormalize CSV line endings under .gitattributes"
```

（`docs/evidence/*.png` と `src/amr_vision/models/trash_yolov8n.pt` も
0x0D を含むが、NUL を含むため git はバイナリと判定する。触らなくてよい。)

## 5. 大きいアセットの方針

現時点でリポジトリは約 7.3MB。**Git LFS はまだ導入しない。**

| 種別 | 扱い | 理由 |
|---|---|---|
| `.f3d` (Fusion native) | 通常 git | **CAD master**。192KB と小さい |
| `.step` | 通常 git | `.f3d` からの派生エクスポート。ASCII |
| `.stl` (`_individual_parts/`) | 通常 git | `pack.py` の入力 |
| `.stl` (`A1_256/` `K1Max_300/`) | 通常 git | `pack.py` の出力（再生成可能） |
| YOLO 重み | `.gitignore` | 過去に 1.1GB で push が弾かれた事故あり |

LFS を検討する条件：単一ファイルが 50MB を超える、または
`6-arm-roboto/` が数百MB 規模になったとき。

## 6. CAD の同時編集をしない

`.step` / `.f3d` は自動マージできない。**同じ CAD ファイルを両 OS で
同時に編集しない。** CAD の編集権は Windows 側にあるものとする。

衝突したら、どちらが canonical かを人が判断する。機械的に片側を採用しない。

## 7. ソースとエクスポートの分離

```
6-arm-roboto/3Dmodel/*.f3d     ← master (Fusion)
6-arm-roboto/3Dmodel/*.step    ← 派生 export
ATN_print/_individual_parts/   ← pack.py の入力
ATN_print/A1_256/              ← pack.py の出力 (Bambu A1, 256mm bed)
ATN_print/K1Max_300/           ← pack.py の出力 (Creality K1 Max, 300mm bed)
```

出力は再生成できる。印刷前に必ず `pack.py` を回して最新化すること。

```bash
python pack.py                       # ATN_print/ を自動解決
ATN_PRINT_DIR=/path/to/out python pack.py   # 出力先を変えたいとき
```

## 8. 絶対パスを埋め込まない

`C:/...` や `/home/...` をコードに直書きしない。
`pathlib` + `__file__` 相対、または環境変数を使う。

シリアルポートのような機械固有値も同様（Linux `/dev/ttyUSB0` /
Windows `COM4`）。設定に出し、論理デバイス名で扱う。

## 9. ブランチ

OS ごとの恒久ブランチ（`ubuntu-branch` / `windows-branch`）を作らない。
機能単位で切って `main` へ統合する。

```
feature/windows-cad-print-firmware
feature/diff-drive
```

## 10. Windows へ新しく clone する手順

```bash
git clone git@github.com:hattoir/auto-trash-navigator.git
cd auto-trash-navigator
git fetch --all --tags --prune
git status          # クリーンであることを確認
python pack.py      # 印刷パイプラインが動くかの最小確認
```

ROS 2 の `src/` は Windows では**ビルドしない**。CAD / 印刷 / ファーム
だけを触る。

## 11. ファイル名

Windows で使えない文字（`: * ? " < > |`）を使わない。
大文字小文字だけが違うファイルを同時に置かない。
（2026-09-14 時点で違反なしを確認済み。日本語ファイル名は両 OS で問題ない。）
