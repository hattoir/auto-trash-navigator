# GPU描画修復 作業記録

作業ブランチ: `feature/gpu-rendering`(mainには一切触れていない)

## 判定: システム設定変更は不要(原因 D)

診断の結果、ドライバ・カーネルモジュールは完全に正常動作していた:

- `nvidia-smi`: 成功。Driver 580.159.03、NVIDIA GeForce RTX 3050 認識
- `lsmod`: `nvidia` `nvidia_drm` `nvidia_modeset` `nvidia_uvm` 全てロード済み
- `dpkg`: `nvidia-driver-580` 等、ドライバパッケージ群は完備

一方、リポジトリの `src/amr_bringup/launch/gazebo.launch.py` と
`display.launch.py` が以下を明示的に強制していた:

- `LIBGL_ALWAYS_SOFTWARE=1`
- `MESA_LOADER_DRIVER_OVERRIDE=llvmpipe`
- `__EGL_VENDOR_LIBRARY_FILENAMES` を mesa の EGL ICD json に固定
- `__GLX_VENDOR_LIBRARY_NAME=mesa`
- サーバープロセスに `LD_PRELOAD=libhide_gpu.so`(GPU隠蔽ライブラリ)を注入

→ **システム側は何も壊れておらず、リポジトリの環境変数だけがソフトウェア
レンダリングを強制していた(原因D)**。そのため、本作業では sudo を要する
システム変更は一切行っていない。dmesg 等の確認もsudo権限がなく実施不可
だったが、上記の状況証拠から原因Dの判定に不整合はないと判断した。

## 追加で判明した事実: Optimus/PRIME構成

この機体は Intel iGPU が物理ディスプレイ(DISPLAY=:0)を駆動し、NVIDIA
RTX 3050 は PRIME render offload 経由でのみ描画に使えるハイブリッド構成
だった。ソフトウェアレンダリング強制を単に外しただけでは GLX クライアント
(RViz2, Gazebo GUI)が Intel iGPU (`iris` ドライバ)に流れてしまうことを
`LIBGL_DEBUG=verbose` で確認した(software/llvmpipeではなく実ハードウェア
だが、NVIDIA ではなかった)。

`__NV_PRIME_RENDER_OFFLOAD=1` と `__GLX_VENDOR_LIBRARY_NAME=nvidia` を
明示することで、`libGLX_nvidia.so` がロードされ、`nvidia-smi` の
Processes 欄に対象プロセスが GPU メモリ使用量とともに表示されることを
確認した(rviz2: 14MiB、gazebo_server: 46MiB、gazebo_gui: 88MiB)。

EGL(Gazeboのヘッドレスサーバー側)は上書き不要だった。
`/usr/share/glvnd/egl_vendor.d/` に `10_nvidia.json` と `50_mesa.json` が
両方存在し、明示的にmesa側を強制しない限りGLVNDがnvidiaのICDを自然に
選択したため。

## システム設定変更: なし

本作業で `sudo` を要する変更、`/etc` 配下の設定変更、パッケージの
インストール・削除は一切行っていない。glxinfo(mesa-utils)のインストール
も見送り、代替として `nvidia-smi` のプロセス一覧と `/proc/<pid>/maps` の
共有ライブラリ、`LIBGL_DEBUG=verbose` の出力で検証した。

「変更前の状態 / 実行したコマンド / 戻し方」の3点セットで記録すべき
システム変更は0件。

## リポジトリ側の変更(commit単位)

### `9cd23f2` feat: enable GPU rendering with software_render fallback flag

- `gazebo.launch.py` / `display.launch.py` に `software_render` launch引数を追加
  (デフォルト `false` = GPU使用)
- `software_render:=true` で従来のllvmpipe強制環境変数一式
  (`LIBGL_ALWAYS_SOFTWARE`, `MESA_LOADER_DRIVER_OVERRIDE=llvmpipe`,
  `MESA_GL/GLSL_VERSION_OVERRIDE`, mesa EGL/GLX vendor固定, `LD_PRELOAD`)
  に完全復元できる
- `software_render:=false`(デフォルト)では上記を一切設定せず、代わりに
  `__NV_PRIME_RENDER_OFFLOAD=1` / `__GLX_VENDOR_LIBRARY_NAME=nvidia` を
  設定してNVIDIA RTX 3050へのPRIMEオフロードを明示
- Xvfb(:101)の起動は `software_render:=true` の場合のみに限定(GPU経路では
  実DISPLAY:0を使うため:101は未参照だったことを確認済み。GUI起動テストで
  Xvfb無しでも問題なく動作することを実地検証した)
- 副次的に発見した既存バグ(`display.launch.py` の `robot_description` が
  `ParameterValue` でラップされておらずlaunch自体が例外終了する)を修正。
  mainブランチにも存在する、GPU修復とは無関係の不具合

## 元に戻す方法(このブランチ内での話。システム変更は無いので巻き戻し不要)

- 個々のGazebo/RViz起動を従来のllvmpipe強制に戻したい場合:
  `ros2 launch amr_bringup gazebo.launch.py software_render:=true`
  `ros2 launch amr_bringup display.launch.py software_render:=true`
- 本ブランチの変更自体を取り消したい場合: `git revert 9cd23f2`
  (システム設定は変更していないため、これだけで完全に元の状態に戻る)
