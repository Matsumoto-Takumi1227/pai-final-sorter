# pai_final_sorter — 視覚認識にもとづく仕分けマニピュレーション

松尾・岩澤研究室「Physical AI 応用編1 2026」最終課題。

YOLOv8 で画像中の物体を認識し、その**クラス名に応じて置き場所を変えて**
CRANE+ V2（Gazebo シミュレーション）でピック&プレースする ROS 2 パッケージ。

```
画像 ──YOLOv8──▶ クラス名 ──対応表──▶ 置き場A/B ──逆運動学──▶ 関節角列
                                                      │
                                       JointTrajectory ▼
                                              CRANE+ V2 (Gazebo)
```

## 構成

| ファイル | 役割 |
|---|---|
| `pai_final_sorter/vision.py` | YOLOv8 推論。画面占有面積が最大の物体を「主対象」とする |
| `pai_final_sorter/kinematics.py` | CRANE+ V2 の順運動学・逆運動学（外部依存なしの自作実装） |
| `pai_final_sorter/sorter_node.py` | 認識 → 判断 → 関節軌道のパブリッシュ |
| `models/sorting_box.sdf` | 仕分け対象の立方体（Gazebo へ投入する） |

## 設計上のポイント

1. **主対象の選び方** — 信頼度最大ではなく**バウンディングボックス面積最大**を選ぶ。
   仕分け対象はカメラ手前にあり画面上で大きく写るため。
2. **逆運動学を 1 回だけ解く** — 肩・肘・手首の角度は手先の「半径 r と高さ z」だけで
   決まり、土台の旋回角 q1 には依存しない。そこで q1=0 の断面で解いて q1 を差し替える。
   掴む位置と置く位置で r, z を揃えてあるので、可動域の検証も 1 回で済む。
3. **未知クラスは掴まない** — 対応表に無いクラスは「判断不能」として待機させる。
   判断できないものを無理に動かさない設計は実ロボットの安全上の基本。
4. **グリッパの符号は実測で決めた** — `crane_plus_joint_hand` は正で閉じ、負で開く。
   直感と逆なので、推測で値を決めず `ros2 topic pub` で実際に両方向へ動かして確認した。
5. **可動域チェック** — 指令を出す前に全関節を CRANE+ V2 のサーボ可動域と照合し、
   外れていれば動作を中止する。

## 使い方

### 1. ビルド

```bash
cd ~/pai_ws
colcon build --symlink-install --packages-select pai_final_sorter
source install/setup.bash
```

### 2. Gazebo を起動（GUI のあるデスクトップ側で）

```bash
ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py
```

### 3. 仕分け対象の立方体を投入

```bash
ros2 run ros_gz_sim create -world default \
  -file ~/pai_ws/src/pai_final_sorter/models/sorting_box.sdf \
  -name sorting_box -x 0.15 -y 0.0 -z 0.02
```

### 4. 実行

```bash
# 乗り物が主対象 -> 置き場A
ros2 run pai_final_sorter sorter --ros-args -p image:=$HOME/pai_final/images/bus.jpg

# 人が主対象 -> 置き場B
ros2 run pai_final_sorter sorter --ros-args -p image:=$HOME/pai_final/images/zidane.jpg

# ロボットを動かさずに計算結果だけ確認
ros2 run pai_final_sorter sorter --ros-args -p image:=... -p dry_run:=true
```

## 主なパラメータ

| 名前 | 既定値 | 意味 |
|---|---|---|
| `image` | `~/pai_final/images/bus.jpg` | 入力画像 |
| `model` | `~/pai_final/yolov8n.pt` | YOLOv8 の重み |
| `r_pick` | `0.15` | 掴む／置く位置の水平距離 [m] |
| `z_hover` | `0.060` | 上空の高さ [m]（r=0.15, pitch=-90° での可動限界は 0.063） |
| `z_grasp` | `0.020` | 掴む高さ [m] |
| `bin_a_deg` / `bin_b_deg` | `55` / `-55` | 置き場の旋回角 [deg] |
| `gripper_open` / `gripper_close` | `-0.50` / `0.55` | グリッパ開閉 [rad]。**正で閉じ、負で開く** |
| `dry_run` | `false` | true なら計算のみ |

## 動作環境

- Docker image `airobotbook/ros2-desktop-ai-robot-book-humble`
- ROS 2 Humble / Ignition Gazebo / ultralytics 8.3.74

## 参考

関節名・トピック名・リンク長は書籍『ROS2とPythonで作って学ぶAIロボット入門』付属の
`crane_plus_commander` および `crane_plus` パッケージの仕様に準拠している。
