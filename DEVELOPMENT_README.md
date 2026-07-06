# 自動ゴミ収集ナビゲーター (auto-trash-navigator) 技術仕様書

本仕様書は、自律走行型自動ゴミ収集ロボット（AMR: Autonomous Mobile Robot）のシミュレーションシステムにおけるハードウェア設計、ROS 2 ソフトウェアアーキテクチャ、およびグラフィックス不調環境向けの起動ワークアラウンドを実ファイル（URDF/Xacro, Launch, Config）の静的解析結果に基づいてドキュメント化したものである。

---

## 1. プロダクト概要 & ハードウェア構成

本プロジェクトのロボットモデルは、4輪メカナム駆動ベース、SO-101 6自由度アーム、平行開閉型グリッパー、および環境認識用センサー（OAK-D RGB-Dカメラ、IMU）を搭載した自律ロボットである。

### 1.1 ロボットモデルの全体構造とリンク構成
ロボットの物理構造および座標リンク構成は、[amr_robot.urdf.xacro](file:///home/pakku/auto-trash-navigator/src/amr_description/urdf/amr_robot.urdf.xacro) で以下のように定義されている。

- **`base_footprint`**: 接地平面上にあるロボットの原点となる仮想基準リンク。
- **`base_joint`**: `base_footprint` から `base_link` へ接続する固定ジョイント（高さ 0.1m）。
- **`base_link`**: ロボット筐体本体の主要部。サイズ `0.4 x 0.4 x 0.15 m` の直方体で定義され、質量は `10.0 kg`。
- **`oak_d_joint`**: `base_link` の前方（`x = 0.18 m`, `z = 0.1 m`）に配置された固定ジョイント。カメラ角度を水平からピッチ方向に `0.26 rad` (約15度) 傾けて固定されている。
- **`arm_base_joint`**: アーム台座を固定するジョイント（`x = 0.1 m`, `z = 0.075 m`）。
- **`arm_base_link`**: アームの土台。半径 `0.05 m`、高さ `0.04 m` の円柱形状で、質量は `0.5 kg`。

```mermaid
graph TD
    base_footprint -->|base_joint (fixed)| base_link
    base_link -->|front_left_wheel_joint| front_left_wheel_link
    base_link -->|front_right_wheel_joint| front_right_wheel_link
    base_link -->|rear_left_wheel_joint| rear_left_wheel_link
    base_link -->|rear_right_wheel_joint| rear_right_wheel_link
    base_link -->|oak_d_joint (fixed)| oak_d_link
    base_link -->|arm_base_joint (fixed)| arm_base_link
    arm_base_link -->|joint1 (revolute)| link1
    link1 -->|joint2 (revolute)| link2
    link2 -->|joint3 (revolute)| link3
    link3 -->|joint4 (revolute)| link4
    link4 -->|joint5 (revolute)| link5
    link5 -->|joint6 (revolute)| link6
    link6 -->|wrist_camera_joint (fixed)| wrist_camera_link
    link6 -->|gripper_base_joint (fixed)| gripper_base_link
    gripper_base_link -->|left_finger_joint (prismatic)| left_finger_link
    gripper_base_link -->|right_finger_joint (prismatic)| right_finger_link
```

### 1.2 4輪メカナムホイールの幾何学的配置と摩擦特性
全方向移動性能を支える駆動ベースは [base_omni_4wheel.xacro](file:///home/pakku/auto-trash-navigator/src/amr_description/urdf/base_omni_4wheel.xacro) によりマクロ化され、呼び出されている。

- **車輪仕様**: 半径 `0.05 m`、幅 `0.04 m` の円柱で、質量は `0.2 kg`。
- **配置レイアウト**:
  - `front_left` : `x = 0.15 m`, `y = 0.18 m`, `z = -0.05 m`
  - `front_right`: `x = 0.15 m`, `y = -0.18 m`, `z = -0.05 m`
  - `rear_left`  : `x = -0.15 m`, `y = 0.18 m`, `z = -0.05 m`
  - `rear_right` : `x = -0.15 m`, `y = -0.18 m`, `z = -0.05 m`
  - ホイールベース（軸距）：`0.30 m`、トレッド（輪距）：`0.36 m`
- **メカナム駆動用の摩擦特性 (`fdir1`)**:
  メカナムホイールの傾斜ローラーの方向を示す `fdir1` 摩擦ベクトルが、各車輪ジョイントのローカル座標に対して以下のように正確に定義されている。
  - `front_left`: `fdir="1 -1 0"`
  - `front_right`: `fdir="-1 -1 0"`
  - `rear_left`: `fdir="1 1 0"`
  - `rear_right`: `fdir="-1 1 0"`
- **Gazebo Sim 物理プラグイン**:
  `gz-sim-mecanum-drive-system` プラグインを使用して、`/cmd_vel` トピックで速度指令を受け取り、車輪分離距離 `0.36 m` とホイールベース `0.30 m` に基づくキネマティクス演算により車輪を回転させ、オドメトリ情報 `/odom` を 50Hz でパブリッシュする。

### 1.3 SO-101 6自由度アームおよびパラレルグリッパーのジョイント構成
アームおよびグリッパーは [so101_arm.xacro](file:///home/pakku/auto-trash-navigator/src/amr_description/urdf/so101_arm.xacro) および [gripper.xacro](file:///home/pakku/auto-trash-navigator/src/amr_description/urdf/gripper.xacro) で構成されている。

- **6自由度アームジョイント仕様**:
  すべての関節が回転型（`revolute`）で、可動範囲の上限/下限は `-2.0 rad` 〜 `2.0 rad`、最大許容トルク `10.0 N・m`、最大角速度 `2.0 rad/s` で統一されている。
  1. `joint1` (軸: `0 0 1` - 旋回)
  2. `joint2` (軸: `0 1 0` - 肩ピッチ)
  3. `joint3` (軸: `0 1 0` - 肘ピッチ)
  4. `joint4` (軸: `0 0 1` - 手首ロール)
  5. `joint5` (軸: `0 1 0` - 手首ピッチ)
  6. `joint6` (軸: `0 0 1` - 手先ロール)
- **手先カメラの設置**: `link6` に固定された `wrist_camera_link` により、アーム視点での近接センシングが可能。
- **パラレルグリッパー仕様**:
  `gripper_base_link` に対して、左右対称に移動する2基の直動型（`prismatic`）ジョイントが対向して配置されている。
  - `left_finger_joint`（軸: `0 -1 0`、可動域: `0.0` 〜 `0.015 m`）
  - `right_finger_joint`（軸: `0 1 0`、可動域: `0.0` 〜 `0.015 m`）
  - 合計で `0.0` 〜 `0.03 m`（最大開幅 30mm）の開閉幅を制御する。最大許容努力値は `10.0 N`、最高速度は `0.1 m/s`。

---

## 2. ROS 2 ソフトウェアアーキテクチャ

本システムは、Gazebo Sim による3次元シミュレーション空間と ROS 2 の制御スタックがトピック通信を介してシームレスに結合されている。

```
+------------------------------------+
|            Gazebo Sim              |
| (Physics Engine, Sensors, Plugins) |
+--+-------+-------+--------------+--+
   ^       |       |              |
   |       |       |              | (Joint / Link States)
   |       |       |              v
   |       |       |     +-------------------------+
   |       |       |     |  robot_state_publisher  |
   |       |       |     +------------+------------+
   |       |       |                  |
   |       |       |                  | (TF: odom -> base_footprint)
   |       |       |                  v
   |       |       |     +------------+------------+
   |       |       |     |     ekf_filter_node     |
   |       |       |     |  (robot_localization)   |
   |       |       |     +------------+------------+
   |       |       |                  ^
   |       |       |                  |
   |       |       |                  | (Fused Odom / Filtered)
   |       |       +----------+       |
   |       | (IMU: /imu)      | (Odom) |
   |       v                  v       |
+--+-------+------------------+-------+--+
|           ros_gz_bridge                |
|         (parameter_bridge)             |
+--+---------------+------------------+--+
   ^               |                  |
   |               |                  | (Depth Image: /camera/depth_image_raw)
   |               |                  v
   |               |         +--------+----------------+
   |               |         | depthimage_to_laserscan |
   |               |         +--------+----------------+
   |               |                  |
   |               |                  | (2D Scan: /scan)
   |               v                  v
   |       +-------+------------------+--+
   |       |        Navigation2 /        |
   |       |        SLAM Nodes           |
   |       +-------+---------------------+
   |               |
   |               | (Cmd_Vel)
   +---------------+
```

### 2.1 ノードとトピックのデータフロー
主要ノードおよび通信トピックの関係は、[gazebo.launch.py](file:///home/pakku/auto-trash-navigator/src/amr_bringup/launch/gazebo.launch.py) で以下の通りに構築されている。

- **`robot_state_publisher`**
  - URDF/Xacro ファイルを動的にパースし、ロボットのリンク関係・座標変換（TF）を生成・発信する。
- **`ros_gz_bridge`** (`parameter_bridge`):
  - ROS 2 と Gazebo Sim 間のデータ相互中継を担う。ブリッジ変換定義は以下の通り。
    - `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock` (時間同期)
    - `/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist` (移動指令: 双方向)
    - `/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry` (車輪オドメトリ: 双方向)
    - `/imu@sensor_msgs/msg/Imu[gz.msgs.IMU` (IMUセンサー)
    - `/camera/image` -> `/camera/image_raw` (カメラ画像)
    - `/camera/depth_image` -> `/camera/depth_image_raw` (デプス画像)
    - `/camera/camera_info` (カメラパラメータ)
    - `/world/empty/model/visual_amr/joint_state` -> `/joint_states` (関節角度情報)
- **`depthimage_to_laserscan_node`** (Phase 2-A):
  - RGB-Dカメラから得られるデプス画像トピック `/camera/depth_image_raw` を、2Dレーザースキャン（LiDAR代替データ）`/scan` にリアルタイムで変換する。
  - **主要パラメータ**:
    - `scan_height` (走査高さ): `15`
    - `range_min` (最小検知距離): `0.3 m`
    - `range_max` (最大検知距離): `8.0 m`
    - `scan_time` (走査周期): `0.033 s` (30Hz)
    - `output_frame` (出力座標基準): `oak_d_link`
- **`ekf_filter_node`** (`robot_localization`) (Phase 2-B):
  - 車輪オドメトリとIMUデータのセンサーフュージョンを行い、高精度な自己位置情報を計算・TFブロードキャストする。
  - **設定値 (`ekf.yaml`)**:
    - `frequency`: `30.0` (Hz)
    - `two_d_mode`: `true` (2次元平面移動用)
    - `publish_tf`: `true` (TF `odom` -> `base_footprint` の書き出しを有効化)
    - `odom0`: `/odom`
      - 使用パラメータ: `vx`（X方向速度）, `vy`（Y方向速度）, `vyaw`（Yaw角速度）
    - `imu0`: `/imu`
      - 使用パラメータ: `yaw`（絶対角度）, `vyaw`（Yaw角速度）

### 2.2 ros2_control による制御マネージャーとコントローラ構成
[amr_controllers.yaml](file:///home/pakku/auto-trash-navigator/src/amr_description/config/amr_controllers.yaml) に基づき、`controller_manager` が関節のフィードバック制御と指令値分配を管理する。

- **`update_rate`**: `100 Hz`
- **`joint_state_broadcaster`**:
  すべての可動関節（`joint1`〜`joint6`、`left_finger_joint`、`right_finger_joint`）の状態を読み取り、ROS 2 上の共通トピック `/joint_states` に配信する。
- **`arm_controller`** (`joint_trajectory_controller/JointTrajectoryController`):
  アームの6関節を位置・速度フィードバックによって制御し、目標とする関節軌道を滑らかに追従させる。
- **`gripper_controller`** (`joint_trajectory_controller/JointTrajectoryController`):
  平行グリッパーの左右の指を連動させ、高精度な開閉位置制御を可能にする。

---

## 3. グラフィックス不調環境における起動ワークアラウンド

本プロジェクトでは、GPUハードウェアアクセラレーションが使用できない仮想環境（VirtualBox, VMware, クラウドインスタンス等）や、X11/OpenGLドライバが不十分な環境でも、Gazebo Sim (GUI) と RViz2 をエラーなく同時起動させるための堅牢な回避策を実装している。

### 3.1 Mesaソフトウェアラスタライザ (llvmpipe) の強制
[gazebo.launch.py](file:///home/pakku/auto-trash-navigator/src/amr_bringup/launch/gazebo.launch.py) 内の `gazebo_env` 辞書、および [display.launch.py](file:///home/pakku/auto-trash-navigator/src/amr_bringup/launch/display.launch.py) 内の `os.environ` を通じて、以下のMesaドライバ指定用環境変数を密輸・強制適用している。

- **`MESA_LOADER_DRIVER_OVERRIDE`**: `llvmpipe` (CPUによる描画処理の強制)
- **`LIBGL_ALWAYS_SOFTWARE`**: `1` (ハードウェアGPUアクセラレーションを明示的に無効化)
- **`__GLX_VENDOR_LIBRARY_NAME`**: `mesa` (ベンダーとしてMesa GLXを指定)
- **`__EGL_VENDOR_LIBRARY_FILENAMES`**: `/usr/share/glvnd/egl_vendor.d/50_mesa.json` (EGLドライバのパス指定)
- **`MESA_GL_VERSION_OVERRIDE`**: `4.5` / **`MESA_GLSL_VERSION_OVERRIDE`**: `450` (OpenGL 4.5/GLSL 4.50として偽装させ、古いハードウェアでもシェーダ読み込みエラーを防ぐ)
- **`QT_QPA_PLATFORM`**: `xcb` / **`GDK_BACKEND`**: `x11` (GUIライブラリがWaylandではなくX11サーバと安全に通信するように強制)

### 3.2 子プロセス（ExecuteProcess）への環境変数密輸
ROS 2 の標準 Launch ノードは自身の環境変数を継承するが、別プログラムである `gz sim` に対して環境変数を100%確実に引き渡すために、`ExecuteProcess` の `additional_env` パラメータを使用している。さらに、シェル展開によるパスの変質や変数の欠落を防ぐため、`shell=False` 相当（コマンド引数をリスト `cmd=[...]` で渡す形式）で分離して実行している。

### 3.3 OGRE 1.x へのフォールバック (Ogre 2.x クラッシュの回避)
Gazebo Sim（旧Ignition）は描画バックエンドとして標準で Ogre 2.x を使用するが、これはソフトウェアレンダリング環境下でセグメンテーションフォールトや表示の乱れを引き起こす。
本構成では、GUI起動時に `--render-engine ogre` を引数として指定し、Ogre 1.x へ強制フォールバックさせることで、描画処理を安定化させている。

```python
    # gazebo.launch.py 内の GUI起動プロセス定義
    gazebo_gui = ExecuteProcess(
        cmd=[
            'ruby', '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz', 'sim',
            '--render-engine', 'ogre',  # OGRE 1.x の強制
            '-r', 'empty.sdf',
            '--force-version', '8'
        ],
        name='gazebo',
        output='screen',
        additional_env=gazebo_env,
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )
```

---

## 4. 開発・生存確認コマンド集

### 4.1 ワークスペースのビルドとソース適用
パッケージが変更された場合、以下のコマンドでビルドを実行し、環境変数をインポートする。
```bash
cd /home/pakku/auto-trash-navigator
colcon build --symlink-install
source install/setup.bash
```

### 4.2 シミュレータ環境 (Gazebo Sim) の起動
物理エンジン、ロボットモデルのスポン、トピックブリッジ、各コントローラ、EKF、およびデプス画像のレーザースキャン変換ノードを一括起動する。
```bash
ros2 launch amr_bringup gazebo.launch.py
```
> [!IMPORTANT]
> **Gazebo 起動時の注意点**
> 起動直後はシミュレーションが**一時停止（Paused）**状態になっている。
> **必ずGazebo GUI画面の下部にある「Play (再生) ボタン（右向きの三角形）」をクリックしてシミュレーションを開始すること。**
> 開始しないと、`/clock` を含む全センサ・オドメトリトピックがROS 2側へ流れない。

### 4.3 センサー情報とロボットの可視化 (RViz2)
ロボット形状、関節姿勢、仮想LiDARスキャンデータ、TFフレーム関係を3次元上で可視化する。
```bash
ros2 launch amr_bringup display.launch.py
```
- 本コマンドを実行すると、[amr.rviz](file:///home/pakku/auto-trash-navigator/src/amr_description/rviz/amr.rviz) 設定ファイルを自動で読み込み、Fixed Frame が `base_footprint` に自動調整された状態で起動する。

### 4.4 キーボードによる全方向移動制御 (teleop)
ロボットの全方向並行移動を確認する。
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- メカナム駆動の特性を活かし、前後のほか、その場での左右スライドや旋回動作が行えることを確認する。

### 4.5 トピックによる生存確認コマンド
起動確認用の補助コマンド一覧。

- **現在有効なトピックの確認**:
  ```bash
  ros2 topic list
  ```
- **融合自己位置推定（EKF）データの監視**:
  ```bash
  ros2 topic echo /odometry/filtered
  ```
- **デプス画像から変換された2Dレーザースキャンの監視**:
  ```bash
  ros2 topic echo /scan
  ```
- **アームおよびグリッパーの関節角度の監視**:
  ```bash
  ros2 topic echo /joint_states
  ```
- **コマンドラインからのアーム動作テスト例**:
  ```bash
  ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  points:
    - positions: [0.5, -0.5, 0.5, 0.0, 0.5, 0.0]
      time_from_start: {sec: 2, nanosec: 0}" -1
  ```
