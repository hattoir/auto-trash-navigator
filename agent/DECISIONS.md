# DECISIONS — auto-trash-navigator

<!-- 将来の自分と将来の Claude が「なぜこうなっているのか」を再構築できるように書く。
     新しい判断は一番上に足す（新しい順）。 -->

## テンプレート

```
## YYYY-MM-DD — <決めたこと>

**Decision**: 何を決めたか
**Why**: なぜそれを選んだか
**Alternatives**: 他に何を検討したか
**Trade-offs**: 何を捨てたか / どういう時に間違いになるか
**Context**: その時点で分かっていたこと
```

---

## 2026-09-14 — Windows 側の未コミット作業を feature ブランチへ退避し、merge はユーザー判断に残した

**Decision**: 約3か月分の未コミット作業（CAD 再エクスポート・印刷プレート・
ESP32 ファーム・手順書）を `feature/windows-cad-print-firmware` に
9 コミットへ分けて記録した。`main` は触っていない。
`origin/main` への merge と push は**実行していない**。

**Why**: 未コミットのまま `git pull` すると 95 コミット分のチェックアウトで
ユーザーの CAD 作業を失う危険があった。まずコミットして可逆にすることは
一方向に安全な操作なので自律実行した。一方 merge は、95 コミット分の
ROS 2 ワークスペース（`src/` `logs/` `maps/` `legacy/`）をこの Windows
フォルダへ実体化させる。ここでは ROS 2 は動かないので、それを持ち込むかは
技術的事実ではなくワークフローの好みであり、ユーザーの判断に属する。

**Alternatives**: (1) 何もせず報告だけ → 作業が未コミットのまま危険が続く
(2) merge と push まで自律実行 → 共有 remote と作業フォルダを
    問い合わせなしに大きく変えることになる
(3) `git stash` で退避 → 3か月分を stash に置くのは可視性が低く危うい

**Trade-offs**: ブランチが 1 本増える。統合の判断は次のセッションへ持ち越す。
その代わり、どの時点にも `git reset --hard <sha>` で戻れる。

**Context**: `git merge-tree` による dry-run で `HEAD` と `origin/main` の
merge は**衝突ゼロ**であることを確認済み。`origin/main` は 95 コミットの間
`6-arm-roboto/` を一切触っていないため、CAD の変更はそのまま生き残る。

---

## 2026-09-14 — .gitattributes を追加し、改行とバイナリを明示した

**Decision**: index は LF、working tree は OS ネイティブ。`.stl` `.3mf` `.f3d`
は binary を明示宣言。`.step` は text（ASCII のため）。

**Why**: `.stl` は `pack.py` が出力する binary STL で、先頭 80 byte が
ASCII ヘッダ（`ATN ...`）。git の自動バイナリ判定は NUL の有無で決まるため
今は正しく binary と判定されているが、暗黙の判定に 7MB の造形データを
預けるのは危うい。明示すれば OS や git の版に依存しなくなる。

**Alternatives**: (1) `core.autocrlf` に任せる（現状）→ 設定が個人の
git config 側にあり、リポジトリを clone しただけでは再現しない
(2) 全部 binary にする → `.step` の差分で再エクスポートを検知できなくなる

**Trade-offs**: `origin/main` 側の `height_scan.csv` / `reach_map.csv` は
index に CRLF で入っているため、合流後に一度だけ `--renormalize` が要る。
`docs/MULTI_OS_DEVELOPMENT.md` §4 にコマンドを書いた。

**Context**: 適用前後で `git status` が変わらないこと（churn ゼロ）を確認済み。

---

## 2026-09-14 — Autonomous Product Development OS を導入した

**Decision**: 共通の作業 OS を `CLAUDE.md` の生成ブロックとして持ち、
プロジェクト固有の状態を `agent/STATE.md` / `ROADMAP.md` / `DECISIONS.md` に分けた。

**Why**: 複数セッションにまたがる作業で、毎回リポジトリを読み直すところから
始まるのを避けるため。共通ルール（OS）と、プロジェクト固有の記憶を分離すると、
OS を 1 箇所で更新でき、記憶はプロジェクトに残る。

**Alternatives**: (1) 何もしない (2) 全部 README に書く
(3) ルートの共通ファイルを `@import` する。

**Trade-offs**: OS 本文が各リポジトリに複製されるので、更新には
`python tools/sync_agent_os.py`（C:/2026 側）の実行が要る。
その代わり、このリポジトリ単体を clone しても OS が欠けない。

**Context**: C:/2026 配下の各プロダクトは独立した git リポジトリ（または未管理）で、
共通の親リポジトリが無い。
