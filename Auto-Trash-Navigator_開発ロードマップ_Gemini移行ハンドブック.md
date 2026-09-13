# Auto-Trash Navigator 開発ロードマップ & Gemini 移行ハンドブック

**作成日**: 2026-07-02
**用途**: 今後の開発工程の全体設計図。Gemini との開発セッション開始時に、このドキュメント(特に §1 と §5)を貼り付けてコンテキストとして使う。

---

## 0. このドキュメントの使い方

1. **新しい Gemini セッションを始めるたびに §1「プロジェクト現状サマリ」を丸ごと貼る**。これで「今どこまで出来ていて、何を壊してはいけないか」を毎回正確に伝えられる。
2. 作業するフェーズの節(§2)を追加で貼り、「今日は STEP X をやる」と宣言する。
3. Gemini の回答が怪しいときは §5「LLM の典型的な間違いリスト」と照合する。
4. 詰まったら §6「リスクと解決策一覧」を先に読む。既知の罠なら答えが載っている。

---

## 1. プロジェクト現状サマリ(Gemini 貼り付け用コンテキスト)

> 以下をそのままコピーして Gemini に渡すこと。

```
【プロジェクト前提 — 必ずこの環境を前提に回答すること】
- ロボット: 自律ゴミ回収AMR「Auto-Trash Navigator」
  (4輪メカナム台車 + 6軸アーム + グリッパー + OAK-D RGB-Dカメラ + IMU)
- OS: Ubuntu 24.04 LTS
- ROS 2 Jazzy Jalisco(ROS 1 の構文・パッケージは一切使わない)
- Gazebo Sim Harmonic(gz-sim)。Gazebo Classic (gazebo11) ではない。
  よって libgazebo_ros_*.so 系プラグイン、spawn_entity.py、gazebo_ros パッケージは全て不正解。
  正解は gz-sim 系プラグイン + ros_gz_bridge / ros_gz_sim。
- 物理: ODE / レンダリング: Ogre2
- 台車: gz-sim MecanumDrive プラグイン(/cmd_vel 受け)
- アーム+グリッパー: gz_ros2_control(arm_controller / gripper_controller)
- センサー: OAK-D 相当(RGB 640x480@30Hz, FOV 71.6°, depth 16-bit) + IMU 50Hz

【完了済み(Phase 1)— 以下は動作検証済みなので変更提案しないこと】
1. gz_ros2_control に <hold_joints>false</hold_joints> を設定済み。
   これを外すと車輪が位置制御でロックされる。絶対に変更しない。
2. 車輪ジョイント軸は <axis xyz="0 ${y_reflect} 0"/>。右輪は180°反転マウント
   (rpy="0 0 pi")のため y_reflect=-1 で極性を相殺している。変更しない。
3. 車輪摩擦は異方性(mu=1.0 / mu2=0.0)。
   <fdir1 gz:expressed_in="base_footprint"> で親座標系にロック済み。
   fdir の符号は FL/RR=(1,-1,0)、FR/RL=(1,1,0)(Pattern B)。変更しない。
4. 車輪の /joint_states は gz-sim-joint-state-publisher-system プラグイン
   + ros_gz_bridge で配信済み。
5. 直進1m誤差0.2%、横行1m誤差0.04%、Yawブレ実質ゼロを実測済み。

【回答時のルール】
- コードは必ず ROS 2 Jazzy / Gazebo Harmonic の API で書く。
- 自信がない API はその旨を明記する。
- 既存の URDF/xacro を書き換える提案をする場合、上の「変更しない」項目に
  触れないことを先に確認する。
- 変更は最小差分で示す(ファイル全体の書き直しをしない)。
```

---

## 2. 開発工程表(Phase 2 〜 完成まで)

全体の流れ:

```
Phase 2: 自律移動基盤(疑似LiDAR → EKF → 地図 → Nav2巡回)
Phase 3: ゴミ認識(YOLO + 3D位置推定 + Nav2一時停止連携)
Phase 4: マニピュレーション(MoveIt 2 + ピック&プレース)
Phase 5: 全体統合(ステートマシンで巡回→発見→回収→再開のループ)
Phase 6: (将来)実機移行
```

**重要な原則: 1フェーズ = 1ブランチ = 1つの動作確認。** 動く状態を git にコミットしてから次へ進む。Phase 1 の完動状態を今すぐ `phase1-complete` タグで固定すること。

---

### Phase 2-A: 疑似 2D LiDAR 化(depthimage_to_laserscan)

**目的**: OAK-D の深度画像を 2D LaserScan (`/scan`) に変換し、Nav2 が使える障害物データを作る。

**作業ステップ**
1. `sudo apt install ros-jazzy-depthimage-to-laserscan`
2. launch に `depthimage_to_laserscan_node` を追加。入力: `/camera/depth_image_raw` と `/camera/camera_info`、出力: `/scan`
3. パラメータ調整: `scan_height`(使う画素行数。10〜20行推奨)、`range_min/range_max`(OAK-D 実効レンジに合わせ 0.3〜8.0m 程度)、`output_frame` はカメラの光学フレーム
4. RViz2 に LaserScan ディスプレイを追加し、壁の形に赤い点列が乗ることを確認

**完了条件**: `ros2 topic hz /scan` が約30Hzで出て、RViz2 上でスキャンが壁と一致する。

**アドバイス / ハマりどころ**
- **光学フレームの回転規約に注意**。ROS のカメラ光学フレームは Z前方・X右・Y下。TF で `camera_link` → `camera_optical_link` の回転(rpy = -π/2, 0, -π/2)が正しく入っていないと、スキャンが天井や床を向く。
- 深度画像のエンコーディングが `16UC1`(mm単位) か `32FC1`(m単位) かを `ros2 topic echo --no-arr` で確認。ノードは両対応だが、range がおかしいときはまずここを疑う。
- **FOV 71.6° しかない**ことを忘れない。360° LiDAR と違い後方・側方は見えない。この制約は Phase 2-D の costmap 設定で対処する(§6-R1)。

---

### Phase 2-B: 拡張カルマンフィルタ(robot_localization)

**目的**: メカナム特有のホイールスリップによる odom 誤差を IMU で補正し、滑らかで信頼できる `odom → base_footprint` TF を作る。

**作業ステップ**
1. `sudo apt install ros-jazzy-robot-localization`
2. `ekf_node` の設定ファイル(ekf.yaml)を作成:
   - 入力1: `/odom`(MecanumDrive のホイールオドメトリ)→ x, y の速度 (vx, vy) を採用
   - 入力2: `/imu`(IMU)→ yaw 角速度と(あれば)姿勢を採用
   - `two_d_mode: true`
   - `publish_tf: true`、`odom_frame: odom`、`base_link_frame: base_footprint`、`world_frame: odom`
3. **重要**: MecanumDrive プラグイン側の TF 配信(odom→base)を止めるか bridge しない設定にする。TF の二重配信は Nav2 を確実に壊す(§6-R4)。
4. 検証: 前進・横行・旋回を行い、`/odometry/filtered` が滑らかに追従することを確認

**完了条件**: TF ツリー(`ros2 run tf2_tools view_frames`)で odom→base_footprint の発行者が ekf_node のみ。横行時のヨードリフトが生値より減っている。

**アドバイス / ハマりどころ**
- EKF の設定行列(どの状態量をどのセンサから採るか)は、**位置は採らず速度を採る**のが定石。odom から x,y 位置を直接採ると共分散が発散しやすい。
- IMU の `frame_id` が URDF のリンク名と一致しているか確認。不一致だと黙って無視される。
- メカナムは横行時に最もスリップする。テストは必ず横行を含めること。

---

### Phase 2-C: 地図生成(slam_toolbox)

**目的**: 巡回エリアの 2D 占有格子地図を作る。

**作業ステップ**
1. `sudo apt install ros-jazzy-slam-toolbox`
2. `online_async` モードで起動(入力: `/scan` + TF)
3. teleop でロボットを操縦してエリアを一周。**狭い FOV を補うため、要所でその場旋回して周囲をスキャンに収める**
4. `ros2 run nav2_map_server map_saver_cli -f my_map` で保存(my_map.pgm / my_map.yaml)

**完了条件**: 保存した地図に壁が二重線・歪みなく写っている。

**アドバイス / ハマりどころ**
- FOV 71.6° の疑似 LiDAR での SLAM はループ閉合が弱い。**ゆっくり動き、こまめに旋回する**のがコツ。歪むならシミュレーション専用に一時的な 360° LiDAR を足して地図だけ作る手もある(地図作成のみの使用なら本番構成を汚さない)。
- 地図の歪みは以降の全フェーズに祟る。ここで妥協しない。

---

### Phase 2-D: Nav2 巡回システム

**目的**: 地図上の Waypoint A→B→C を無限循環する自律走行を実現する。

**作業ステップ**
1. `sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`
2. `nav2_params.yaml` を作成。**メカナム(全方向移動)向けの必須変更点**:
   - AMCL: `robot_model_type: "nav2_amcl::OmniMotionModel"`(デフォルトは差動駆動)
   - コントローラ(MPPI 推奨): 横方向速度を許可(MPPI なら `vy_max` を設定。DWB なら `min_vel_y/max_vel_y` と y 加速度)
   - `robot_radius` またはフットプリントを実寸で設定
3. costmap 設定: global は static layer + obstacle + inflation、local は rolling window。`observation_sources` に `/scan` を登録
4. RViz2 の「Nav2 Goal」で単発ナビ成功を確認
5. `nav2_simple_commander`(Python API)で `followWaypoints()` を使った循環スクリプトを書く。ループ終了後に再度先頭 Waypoint へ送る while ループで無限巡回

**完了条件**: 3点循環を10周以上、スタック・衝突なしで完走。

**アドバイス / ハマりどころ**
- **全ノードに `use_sim_time: true`** を徹底(§6-R5)。Nav2 が動かない原因の第1位。
- 前方しか見えないため、後退や旋回で「見えていない障害物」に当たるリスクがある。対策: costmap の `obstacle_layer` で観測の持続時間 (`observation_persistence`) を数秒に設定して記憶させる、inflation 半径を広めに取る、Nav2 の挙動として後退を抑制する。
- コントローラは Jazzy 世代なら **MPPI コントローラ**が推奨(全方向移動対応が素直)。DWB でも可能だが y 速度対応の設定項目が多い。
- 最初は障害物のない広い環境で成功させ、後から障害物を足す。一度に全部やらない。

---

### Phase 3: ゴミ認識と Nav2 一時停止連携

**目的**: 巡回中にゴミを検出し、3D 位置を算出し、手前で停止してアームへ引き継ぐ。

**作業ステップ**
1. **検出モデル**: シミュレーションではまず YOLOv8/v11 系の COCO 学習済みモデル(bottle, cup 等)で開始。Gazebo にペットボトル等の COCO クラスに存在するモデルを置けば追加学習不要
2. **推論ノード**: `/camera/image_raw` 購読 → 2D バウンディングボックス出力。GPU がなければ低解像度化 or 数Hzに間引き
3. **3D 位置化**: BBox 中心画素の depth 値 + `camera_info` の内部パラメータで 3D 点を計算(`image_geometry` の PinholeCameraModel が楽)。`tf2` で `map` 座標へ変換して配信(例: `/detected_trash` PoseStamped)
4. **接近ロジック**: 検出したら Waypoint 巡回タスクを `cancel`、ゴミ手前 0.4〜0.5m の姿勢(ゴミの方向を向く)を計算して `goToPose()`。`nav2_simple_commander` なら数十行で書ける
5. 到着したら「回収要求」トピック/アクションを発行して Phase 4 へバトン

**完了条件**: 巡回中にゴミを置くと、自動で巡回を中断→ゴミ正面 0.5m で静止→(ダミーの)回収完了後、巡回を再開する。

**アドバイス / ハマりどころ**
- BBox 中心 1 画素の depth はノイズで飛ぶ。**BBox 内の depth の中央値**を使う。0 (無効値) の除外を忘れない。
- 画像と depth のタイムスタンプ同期に `message_filters` の ApproximateTimeSynchronizer を使う。
- 同一のゴミを何度も「新規発見」しないよう、検出済みリスト(map座標で半径0.3m以内は同一とみなす等)を持つ。
- 停止距離はアームのリーチから逆算する。先に Phase 4 でアームの可達範囲を測っておくと手戻りがない。

---

### Phase 4: MoveIt 2 によるピック&プレース

**目的**: 停止位置からアームでゴミを把持し、車体後方のダストボックスへ投入する。

**作業ステップ**
1. `sudo apt install ros-jazzy-moveit` → **MoveIt Setup Assistant** で設定パッケージ生成:
   - planning group: `arm`(joint1〜6)、`gripper`(fingers)
   - virtual joint: `base_footprint` を world に対して `planar` で定義(移動台車のため)
   - 自己干渉行列の生成、名前付きポーズ(`home`, `look_down`, `drop_pose`)登録
2. RViz2 の MotionPlanning プラグインで対話的にプランニング→実行が通ることを確認(コントローラは既存の `arm_controller` に接続)
3. グリッパー開閉を単体テスト(`gripper_controller` に開/閉の JointTrajectory を送る)
4. **ピックシーケンス**をノード化:
   `/detected_trash` の 3D 位置 → base 座標へ変換 → プリグラスプ姿勢(対象の上方/手前) → 接近 → 閉爪 → 持ち上げ → `drop_pose` へ移動 → 開爪 → `home`
5. 統合テスト: Phase 3 の停止 → 本シーケンス起動 → 成功/失敗を返す

**完了条件**: 静止状態から、既知位置のペットボトルを 8/10 回以上回収してダストボックスへ投入できる。

**アドバイス / ハマりどころ**
- **6軸アームは逆運動学の解が存在しない姿勢が多い**。まず「アームが届く範囲マップ」を作る(数十点の目標をループで solve して成功率を見る)。届かない位置のゴミは「車体を寄せ直す」制御で解決する方が、アームで無理をするより確実。
- 把持の物理はシミュレーションの鬼門(§6-R8)。指リンクの摩擦を上げる(mu 2.0前後)、接触の `kp/kd` を調整、対象物は軽く(50〜100g)設定。それでも滑るなら、把持判定後に対象をグリッパーへ固定ジョイントで「吸着」させる detachable joint 方式が実用的(実機では通常の把持に戻す)。
- 閉爪は位置制御より **effort 制限付き**で「握り込み」にする。位置目標で閉じ切ると物体を弾き飛ばす。
- MoveIt の planning scene に床と車体を追加しないと、床にぶつかる軌道を平気で生成する。

---

### Phase 5: 全体統合(自律ループ)

**目的**: 「巡回 → 発見 → 接近 → 回収 → 巡回再開」の完全自律ループを安定動作させる。

**作業ステップ**
1. 状態遷移の実装。推奨は **py_trees / py_trees_ros**(ビヘイビアツリー)か、シンプルに Python の状態機械(enum + while ループ)。最初は後者で十分:
   `PATROL → DETECTED → APPROACH → PICK → DEPOSIT → PATROL`(各状態にタイムアウトと失敗時遷移を必ず付ける)
2. 失敗リカバリ設計:
   - APPROACH 失敗(ナビ不能)→ そのゴミをブラックリスト化して巡回再開
   - PICK 失敗 → 1回だけ車体位置を微調整してリトライ → 駄目なら諦めて巡回再開
3. 長時間試験: 30分〜1時間の連続稼働でメモリリーク・ノード死・TF 破綻がないか確認
4. 仕上げ: launch ファイル1本で全システム(Gazebo, bridge, EKF, Nav2, 認識, MoveIt, 統合ノード)が起動するように統合

**完了条件**: ゴミ3個を撒いた環境で、無操作で全回収して巡回に戻る。これが**プロジェクトの完成条件**。

**アドバイス**
- 統合ノードは「賢くしない」。判断ロジックを1箇所に集約し、各機能(Nav2/MoveIt/認識)はアクション・サービスとして呼ぶだけにする。デバッグ性が段違い。
- 各状態遷移を必ずログに出す。「今どの状態で何を待っているか」が見えないと長時間試験のデバッグが不可能になる。

---

### Phase 6(将来): 実機移行の視点

今から意識しておくと後で楽になる点だけ列挙する。

- OAK-D 実機は `depthai-ros`(`ros-jazzy-depthai-ros`)でトピック名・型をシミュレーションと揃えられる。**トピック名を今のうちに実機ドライバの命名に寄せておく**と移行がスムーズ。
- YOLO は OAK-D 内蔵 VPU 上で実行可能(depthai の spatial detection)。実機ではホスト PC の GPU 不要になる構成が狙える。
- メカナムの実機は床材によるスリップがシミュレーションよりずっと大きい。EKF(Phase 2-B)を丁寧に作っておくことが最大の保険。
- `ros2_control` のハードウェアインターフェースを差し替えるだけで上位層(Nav2/MoveIt)がそのまま動くのが ROS 2 の設計思想。**上位層にシミュレーション固有の依存を書かない**こと。

---

## 3. スケジュール目安と依存関係

| フェーズ | 内容 | 目安工数 | 依存 |
|---|---|---|---|
| 2-A | 疑似2D LiDAR | 0.5〜1日 | Phase 1 |
| 2-B | EKF (robot_localization) | 1〜2日 | 2-A不要(並行可) |
| 2-C | SLAM 地図生成 | 0.5〜1日 | 2-A, 2-B |
| 2-D | Nav2 巡回 | 3〜7日(最難関の一つ) | 2-A〜2-C |
| 3 | YOLO + 3D位置 + 停止連携 | 3〜5日 | 2-D |
| 4 | MoveIt 2 ピック&プレース | 5〜10日(最難関) | Phase 1(2系と並行可) |
| 5 | 統合ループ | 3〜5日 | 3, 4 |

**並行のヒント**: Phase 4(MoveIt)は Nav2 と独立なので、Nav2 のパラメータ調整で煮詰まったら Phase 4 に切り替えて気分転換できる。統合(Phase 5)まで両者は交わらない。

---

## 4. 検証コマンド チートシート

LLM の出力を鵜呑みにせず、必ず自分の目で確認する。

```bash
# トピック一覧・型・周波数・実データ
ros2 topic list
ros2 topic info /scan --verbose        # QoS まで表示
ros2 topic hz /scan
ros2 topic echo /odometry/filtered --once

# TF ツリーの健全性(最重要)
ros2 run tf2_tools view_frames         # frames.pdf 生成
ros2 run tf2_ros tf2_echo map base_footprint

# ノード・パラメータ
ros2 node list
ros2 param get /amcl robot_model_type
ros2 param get /controller_server use_sim_time   # sim_time 事故の一発確認

# コントローラ状態
ros2 control list_controllers

# Gazebo 側の生トピック(bridge 前)
gz topic -l
gz topic -e -t /model/amr_robot/odometry

# アクションの手動発行(Nav2 テスト)
ros2 action list
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{...}"
```

---

## 5. Gemini 開発ガイド(ここが本ドキュメントの肝)

### 5-1. セッションの始め方(テンプレート)

毎回、§1 のコンテキストブロックを貼った上で、次の形式で依頼する:

```
上記の前提を踏まえてください。
今日のタスク: Phase 2-A(depthimage_to_laserscan の導入)
やってほしいこと: launch ファイルへの追加差分とパラメータ yaml を提示。
制約: 既存ファイルの全文書き換えは禁止。差分のみ。
出力後に、動作確認用のコマンドも添えてください。
```

### 5-2. LLM(Gemini 含む)の典型的な間違いリスト — 出てきたら即座に疑う

学習データには ROS 1 と Gazebo Classic の情報が大量に含まれるため、**古い世代の回答が混ざるのが最頻出の事故**。以下が出てきたらその回答は破棄して指摘し直すこと。

| 出てきたら間違い | 正しい姿(Jazzy + Harmonic) |
|---|---|
| `gazebo_ros` パッケージ、`libgazebo_ros_camera.so` 等 | `gz-sim` 内蔵プラグイン + `ros_gz_bridge` |
| `spawn_entity.py` | `ros_gz_sim` の `create` |
| `gazebo` コマンド、`.world` を gazebo11 で開く | `gz sim` コマンド、SDF world |
| `rosrun` / `roslaunch` / `catkin_make` | `ros2 run` / `ros2 launch` / `colcon build` |
| `tf` (ROS1) パッケージ | `tf2_ros` |
| `robot_state_publisher` に古い引数形式 | Jazzy の launch 記法(`Node(..., parameters=[...])`) |
| Nav2 の古いパラメータ名(Humble 以前の構成) | Jazzy の nav2_bringup 同梱 params を出発点にする |
| `<gazebo>` タグ内に Classic 形式のセンサ記述 | Harmonic の `<sensor>` + gz 系プラグイン名 |
| MoveIt 1 (`moveit_commander` 等の ROS1 API) | MoveIt 2 (`moveit_py` または C++ `MoveGroupInterface`) |

### 5-3. Gemini との付き合い方 実践ルール

1. **1回の依頼は1ファイル・1機能に絞る**。「Nav2 を全部設定して」は事故のもと。「amcl のパラメータだけ」「controller_server だけ」と刻む。
2. **全文書き換えを禁止し、差分で受け取る**。LLM はファイルを書き直すときに動いていた部分を静かに消す。特に本プロジェクトの URDF は §1 の「変更しない4項目」が命綱。
3. **エラーを貼るときはコンテキスト付きで**。エラー全文 + 実行したコマンド + 関連ファイルの該当部分、の3点セット。エラー1行だけ貼ると推測で答えてくる。
4. **「そのAPIはJazzyに存在しますか?」と確認させる**。怪しいときは「公式ドキュメントのどこに書いてあるか」を尋ねると幻覚が減る。
5. **バージョンを毎回明示する**。会話が長くなると前提を忘れて Humble や Classic の回答に戻ることがある。おかしいと感じたら §1 を貼り直す。
6. **動作確認は必ず自分で**(§4 のチートシート)。LLM は「これで動きます」と言うだけで実行はできない。
7. **うまくいった変更は即コミット**。LLM に次を頼む前に `git commit`。壊されても戻れる状態を常に維持する。
8. 長い調査(パラメータの意味調べ等)は Gemini に、**判断(この設計でいくか)は自分で**。設計判断を丸投げすると一貫性が崩れる。

---

## 6. 心配事(リスク)と解決策 一覧

### R1: FOV 71.6° の疑似 LiDAR では視野が狭すぎる 【発生確率: 高】
- **心配**: 側方・後方が見えず、Nav2 が「見えない障害物」に衝突する。SLAM の地図も歪みやすい。
- **解決策**: ① costmap の obstacle layer に観測の保持(persistence)を設定し「一度見た障害物を覚えさせる」。② inflation 半径を広めに。③ 地図作成時はこまめに旋回。④ どうしても不足なら、シミュレーション上に 360° 2D LiDAR を1本追加するのが最も確実(実機でも安価な 2D LiDAR 追加は現実的な設計変更)。

### R2: Nav2 がメカナムの横移動を活かせない 【発生確率: 高】
- **心配**: デフォルト設定は差動駆動前提。横移動せずに旋回ばかりする。
- **解決策**: AMCL を `OmniMotionModel` に、コントローラを MPPI にして `vy_max` を設定。プランナーは Smac/NavFn どちらでも可だがコントローラ側の y 速度設定が本体。§2 Phase 2-D 参照。

### R3: use_sim_time の不統一 【発生確率: ほぼ確実に一度は踏む】
- **心配**: 一部ノードだけ実時間で動き、TF の時刻が合わず「Lookup would require extrapolation」エラーが多発、Nav2 が沈黙する。
- **解決策**: launch で全ノードに `use_sim_time: true` を渡す。疑ったら `ros2 param get <node> use_sim_time` で全ノードを総点検。launch 冒頭で共通変数化しておくと漏れない。

### R4: TF の二重配信(odom→base) 【発生確率: 高(Phase 2-B で必ず遭遇)】
- **心配**: MecanumDrive プラグインと ekf_node の両方が odom→base_footprint を発行し、ロボットが RViz 上で振動・瞬間移動する。
- **解決策**: TF は必ず一元化。bridge の設定で Gazebo 側の TF を `/tf` に流さない(または別名にする)。`view_frames` で各 TF の発行者を確認する習慣をつける。

### R5: QoS 不一致でトピックが「あるのに届かない」 【発生確率: 中】
- **心配**: センサ系は BEST_EFFORT、購読側が RELIABLE 要求だと接続されず、エラーも出ない。
- **解決策**: `ros2 topic info <topic> --verbose` で両側の QoS を比較。ros_gz_bridge のセンサトピックは QoS 設定を明示できる。「hz は出るのにノードが反応しない」ときはまず QoS。

### R6: 把持がシミュレーションで安定しない 【発生確率: 高(Phase 4 最大の壁)】
- **心配**: 掴んだ物体が滑る・弾ける・貫通する。ODE の接触計算は細い指と軽い物体が苦手。
- **解決策**: ① 指の摩擦係数を上げる ② 物体の慣性を現実的に(軽すぎ・重すぎ両方 NG)③ 物理ステップを小さく ④ effort 制限付きで握り込む ⑤ 最終手段として把持成立後に固定ジョイントで「吸着」する detachable joint 方式。⑤は見た目上完璧に動き、統合(Phase 5)の開発を止めない。物理把持の追求は統合完了後にやる、と割り切るのが工程上は正解。

### R7: YOLO 推論が重くてシミュレーションが遅くなる 【発生確率: 中】
- **心配**: RTF(リアルタイムファクター)が落ち、制御周期が乱れて全体が不安定化。
- **解決策**: 推論を 5Hz 程度に間引く/入力を 320px に縮小/巡回中のみ推論を有効化。GPU があれば TensorRT 化。RTF は `gz stats` で常時監視。

### R8: アームが届かない位置で停止してしまう 【発生確率: 中】
- **心配**: Phase 3 の停止位置と Phase 4 の可達範囲が噛み合わず、統合時に手戻り。
- **解決策**: Phase 4 の序盤で「可達範囲の実測」を行い、その結果から Phase 3 の停止距離・角度を決める(依存の向きを明確に)。届かない場合の「寄せ直し」動作を Phase 5 のリカバリに入れる。

### R9: LLM(Gemini)が古い世代のコードを出す 【発生確率: 極めて高】
- **心配**: ROS 1 / Gazebo Classic の回答が混ざり、存在しないプラグインを延々デバッグさせられる。
- **解決策**: §5-2 の間違いリストを常に横に置く。§1 のコンテキストを毎セッション貼る。パッケージ名が出たら `apt list ros-jazzy-*<名前>*` で実在を即確認。

### R10: 動いていた環境が壊れて戻れなくなる 【発生確率: 中・被害: 甚大】
- **心配**: LLM の提案を試すうちに Phase 1 の完動状態を失う。
- **解決策**: 今すぐ git リポジトリ化して `phase1-complete` タグを打つ。フェーズごとにブランチ、動作確認のたびにコミット。ワークスペース全体(src/)を対象にし、ビルド成果物(build/ install/ log/)は .gitignore。

### R11: 「同じゴミを永遠に拾い続ける」等の統合ロジックバグ 【発生確率: 中】
- **心配**: 回収失敗したゴミに無限に再挑戦してループが止まる。
- **解決策**: 検出物に ID とリトライ回数を持たせ、上限超過でブラックリスト化。全状態にタイムアウトを設ける(§2 Phase 5)。

### R12: バージョン互換の崩壊 【発生確率: 低・被害: 甚大】
- **心配**: `apt upgrade` や別バージョンの Gazebo 混入で環境が壊れる。
- **解決策**: Jazzy + Harmonic は公式サポートされたペアなので、この組み合わせを固定する。`ros_gz` は必ず apt の Jazzy 用バイナリ(ソースビルド混在は避ける)。環境構築手順を README に記録しておき、最悪クリーンインストールで復元できる状態を保つ。

---

## 7. 最後に: 進め方の推奨順序(明日からの To-Do)

1. git リポジトリ化 + `phase1-complete` タグ(30分)← 何よりも先に
2. Phase 2-A: depthimage_to_laserscan で `/scan` を出す(半日)
3. Phase 2-B: EKF 導入と TF 一元化(1〜2日)
4. Phase 2-C: 地図作成(半日)
5. Phase 2-D: Nav2 単発ナビ → Waypoint 循環(1週間目安)
6. 以降、Phase 3 → 4 → 5(4 は 2-D と並行可)

各ステップの開始時に、このドキュメントの該当節 + §1 を Gemini に貼ること。健闘を祈る。この AMR は必ず完全自律で走る。

