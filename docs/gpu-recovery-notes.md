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

## 追加調査: render-engine 'ogre'(Ogre1)→'ogre2'への変更

段階5の色復活試験で、GPU使用が確定した状態(nvidia-smiでメモリ使用・
libGLX_nvidia.so読み込みを確認済み)でも `/camera/image_raw` が全ピクセル
R=G=B の完全グレー(彩度0)であることが判明した。`gz sim --help` で
確認したところ、gz simのデフォルトレンダーエンジンは 'ogre2' であり、
`gazebo.launch.py` が明示的に旧世代の 'ogre'(Ogre1)を指定していた
ことが分かったため、GPU経路(`software_render:=false`、デフォルト)では
'ogre2' を使うよう変更した(`render_engine = 'ogre' if software_render
else 'ogre2'`)。`software_render:=true` の従来経路は 'ogre' のまま維持。

**この変更では色の問題は解決しなかった**: `software_render:=true`
(従来のCPU/Ogre1経路)でも生の gz トピック(`/camera/image`、
`pixel_format_type: RGB_INT8` を確認済み)の段階で既に R=G=B の完全グレー
であることを直接確認した(赤球テストで ambient/diffuse=(1,0,0,1) を
設定しても変化なし)。つまりこの「灰色描画」問題は **CPU/GPU、Ogre1/Ogre2
のいずれとも無関係な、別の原因(材質/シェーディング/センサパイプライン
のいずれか)による、修復前から存在する問題**である。

'ogre2'への変更自体は有害ではない(gz simの推奨デフォルトに戻すだけであり、
段階5のRTF・非退行スモークテストで問題は確認されなかった)ため、
GPU経路のデフォルトとして採用し commit した。ただし段階5の「色の復活」
成功基準(彩度≥100)は本タスクの範囲内では達成できていない。

## 未解決の問題(スコープ外として報告)

「/camera/image_raw が常に無彩色になる」問題は、GPUレンダリング復元
(本タスクの主題)とは独立した別バグであり、今回は原因未特定のまま
報告に留める。

### 追加調査(最大2ステップ、2026-07-29実施)

**ステップ1: GUIとセンサーの比較(決定実験)**

GPU描画有効(`software_render:=false`、`ogre2`)の状態で
`ros2 launch amr_bringup gazebo.launch.py headless:=false` を起動し、
視覚のみの赤球(ambient/diffuse=(1,0,0,1))をワールド中央にspawnして
比較した。

- Gazebo GUI(`/gui/screenshot` サービスで取得):
  球は赤色、背景の障害物ボックス(ピンク・黄色)も正しい色で表示。
  → [docs/evidence/gui_screenshot_red_ball.png](evidence/gui_screenshot_red_ball.png)
- 同時刻の `/camera/image_raw`(ロボット搭載カメラ):
  最大彩度 0.00%、全ピクセル R=G=B(平均0.71程度)の完全グレー。
  → [docs/evidence/sensor_frame_gray.png](evidence/sensor_frame_gray.png)

**結果: GUI=赤 / センサー=グレー → 判定は「センサーのレンダーパス側の
問題」(ステップ2A)。** マテリアル定義・SDF解析自体は正常(GUIが正しく
色を再現できている以上、シーン層・マテリアル層は無罪)。

**ステップ2A: センサーパス側の調査**

1. `src/amr_description/urdf/amr_robot.urdf.xacro` の
   `gz::sim::systems::Sensors` プラグイン設定を確認したところ、
   `<render_engine>ogre2</render_engine>` が明示的に指定されていた
   (CLI引数 `--render-engine-server`/`--render-engine-gui` とは独立した、
   センサー専用のレンダーエンジン指定)。ソフトウェアレンダリング経路
   (`software_render:=true`、CLI側はogre1)でもこのタグは変更していない
   ため、その経路では実質「メインシーン=ogre1 / センサー=ogre2」という
   不一致な組み合わせで動いていたことになる。GPU経路(ogre2/ogre2で一致)
   でもグレーが再現したため、エンジンの不一致だけが原因ではない。

2. URDFのカメラセンサー定義(`<camera><image><format>R8G8B8</format>`)
   は正しいことを既存の調査で確認済み(本ドキュメント冒頭の記録より)。

3. `libgz-sim8-sensors-system.so.8.11.0`(vendorビルドのバイナリのみで
   ソース非公開)のシンボルを `strings` で確認したところ、以下を発見:
   - `gz::sim::v8::components::Component<bool, RenderEngineServerHeadlessTag, ...>`
     というECSコンポーネントが存在する
   - `gz::sim::v8::RenderUtil::SetHeadlessRendering(bool const&)` という
     メソッドが存在する
   - これらは、サーバー側(`--headless-rendering` 指定時)のレンダリングが
     GUIプロセスの対話的レンダリングとは明確に区別された、専用の
     "ヘッドレス"状態フラグ・コードパスを持つことを示す
   - `background_color` / `ambient_light` / `global_illumination` という
     文字列も同バイナリ内に存在するが、これらはGUIプラグインのSDF
     パラメータ名の並びに近く、Sensorsシステム自体の設定項目である
     確証は得られなかった
   - grayscale/monochrome/luminance等を示唆するシンボルは見つからず、
     色変換ロジックの直接的証拠は得られなかった

   → **公式ドキュメント/ソースでの確証までは至らなかった**(ROS
   jazzy用vendorパッケージはヘッダ+バイナリのみでgz-sim本体のソースを
   含まず、CHANGELOGの類も同梱されていない)。ただし、GUI(別プロセスの
   独立したOgre2インスタンス)とサーバー内蔵のSensorsシステム(これも
   Ogre2だが `--headless-rendering` 経由の別のレンダリングコンテキスト)
   が完全に別の描画パスであることはアーキテクチャ上確実であり、
   「ヘッドレス専用コードパスが何らかの理由で色情報を落としている」
   という仮説は、今回得られた状況証拠(GUI=正常、ヘッドレスセンサー=
   グレー、ヘッドレス専用ECSコンポーネントの存在)と矛盾しない。

**打ち切り(2ステップ上限に到達)。次に調べるべき候補:**
- `--headless-rendering` を外した非ヘッドレス構成(server/guiを分離せず
  単一プロセスで `gz sim -r` のみ起動)でセンサー画像に色が出るか比較する
  (ヘッドレス専用コードパス仮説の直接検証)
- gz-sim本体(vendorでなく公式リポジトリ)のソースで
  `RenderUtil::SetHeadlessRendering` 呼び出し箇所と、rgbd_cameraセンサーの
  ヘッドレス時のシーン共有処理を確認する
- `--render-engine-server-api-backend` (opengl/vulkan等)の違いが
  センサー側にのみ影響していないか確認する
- gz-simのGitHub Issueで "headless" "camera" "gray"/"grayscale" 等の
  既知不具合を検索する(本セッションはネット非接続のため未実施)
