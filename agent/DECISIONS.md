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
