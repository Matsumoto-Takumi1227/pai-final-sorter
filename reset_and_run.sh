#!/bin/bash
# 仕分けデモを初期状態からやり直すためのスクリプト。
#   使い方: ./reset_and_run.sh [画像パス] [追加の --ros-args ...]
#   例:     ./reset_and_run.sh ~/pai_final/images/bus.jpg -p gripper_close:=0.45
#
# やっていること:
#   1. アームを退避姿勢へ戻す（箱の真上に居座っていると箱を出せないため）
#   2. 前回の箱を消して、同じ場所に置き直す
#   3. 仕分けノードを実行する
set -u

IMAGE="${1:-$HOME/pai_final/images/bus.jpg}"
shift || true

echo "--- 1. アームを退避 ---"
ros2 topic pub --once /crane_plus_arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['crane_plus_joint1','crane_plus_joint2','crane_plus_joint3','crane_plus_joint4'],
    points: [{positions: [0.0,-1.16,-2.01,-0.73], time_from_start: {sec: 3}}]}" >/dev/null
sleep 4

echo "--- 2. 箱を置き直す ---"
ign service -s /world/default/remove --reqtype ignition.msgs.Entity \
  --reptype ignition.msgs.Boolean --timeout 2000 \
  --req 'name: "sorting_box", type: MODEL' >/dev/null 2>&1
sleep 1
ros2 run ros_gz_sim create -world default \
  -file "$HOME/pai_ws/src/pai_final_sorter/models/sorting_box_small.sdf" \
  -name sorting_box -x 0.15 -y 0.0 -z 1.06 >/dev/null 2>&1
sleep 2

echo "--- 3. 仕分け実行: $(basename "$IMAGE") ---"
ros2 run pai_final_sorter sorter --ros-args -p image:="$IMAGE" "$@"
