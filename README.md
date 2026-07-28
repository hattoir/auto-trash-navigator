# auto-trash-navigator

ROS 2 + Gazebo 上で動作する、自律ゴミ回収 AMR(Autonomous Mobile Robot)。
オフィス空間を巡回して紙くずを検出し、アームで回収してダストボックスへ
運搬する一連の自律動作を、シミュレーション環境で完成させたプロジェクト。
2026-07-24 時点で無介入20分の統合検証に合格した完成状態
(タグ `project-complete-verified`)。

## 構成

- **台車**: メカナム4輪駆動(オムニホイール構成)
- **アーム**: SO-101 6軸アーム + グリッパ
- **センサ**: RGB-D カメラ(OAK-D 相当)、2D LiDAR、IMU

## 技術スタック

- ROS 2 Jazzy
- Gazebo Harmonic(gz sim)
- Nav2(AMCL によるローカリゼーション + MPPI Omni コントローラ)
- slam_toolbox
- MoveIt 2(アーム軌道計画・IK)
- 深度画像ベースの自己較正式トラッシュ検出(床平面フィッティング +
  色フィルタ + 既知位置ホワイトリストによる誤検出棄却)

## ディレクトリ構成

```
src/                 現行 AMR ワークスペース(colcon パッケージ群)
  amr_bringup/        launch, world, patrol/pick_and_place/検出器スクリプト
  amr_description/    URDF/xacro(台車・アーム・センサ)
  amr_control/        コントローラ設定
  amr_moveit_config/  MoveIt 2 設定(SRDF, kinematics, planning)
  amr_vision/         ビジョン関連パッケージ
legacy/               旧 minicar ワークスペース(colcon ビルド対象外。
                       legacy/COLCON_IGNORE により除外)
6-arm-roboto/         実機アームの CAD(3Dモデル・図面)
tools/                評価・キャリブレーション・デバッグ用の単発スクリプト
docs/                 開発ログ・詳細ドキュメント
logs/                 走行評価ログ(CSV)
maps/                 SLAM 生成済みマップ(実行時に launch から絶対パス参照)
```

上記のうち `fastdds_udp_only.xml` / `image_synchronizer.py` / `maps/` /
`src/amr_bringup/scripts/reach_map.csv` は launch ファイルから絶対パス
(`/home/pakku/auto-trash-navigator/...`)で参照されているため、
リポジトリ内での配置を変更できない。

## 起動方法

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch amr_bringup full_autonomy.launch.py
```

`colcon build --symlink-install` でビルド(`legacy/` は COLCON_IGNORE に
より対象外)。

## シミュレーション専用の割り切り

現状の完成形は Gazebo 上での動作検証を優先しており、以下はシミュレーション
専用の実装で実機とは非互換:

- **把持**: DetachableJoint ではなく `gz service set_pose` によるテレポート
  方式(アームの接近・軌道は本物だが、最終的な「掴む/離す」はシム内の
  座標書き換えで実現)
- **紙くず**: 静的(static)な視覚オブジェクト。物理干渉・転がりは再現しない
- **検出**: 深度カメラによる床平面フィッティング検出に加えて、既知の
  スポーン位置ホワイトリストで誤検出を棄却する設計(実機では使えない仮定)

## 既知の課題(実機移行時の宿題)

- URDF と Fusion 360 実寸(`6-arm-roboto/`)の整合確認
- 色フィルタベースの検出を YOLO 等の学習ベース検出に置き換え
- テレポート把持を実際のグリッパ制御・力覚フィードバックに置き換え
- GPU レンダリング環境の修復(現状は `hide_gpu` 等の回避策に依存)
- launch ファイルが抱える絶対パス依存(`/home/pakku/auto-trash-navigator/...`)
  の解消
