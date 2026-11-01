"""視覚認識にもとづく仕分けマニピュレーション（Physical AI 応用編1 最終課題）.

処理の流れ:
    画像 --YOLOv8--> クラス名 --対応表--> 置き場 --逆運動学--> 関節角列
        --JointTrajectory--> CRANE+ V2 (Gazebo)

「認識 -> 判断 -> 動作」という Physical AI の基本ループを、
すべて Python で、外部の大きなフレームワークに頼らず組んだもの。
"""

import os
import threading
import time
from math import degrees, pi, radians

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pai_final_sorter.kinematics import (
    gripper_in_range, inverse_kinematics, joint_in_range)
from pai_final_sorter.vision import detect_main_object

# --- クラス名 -> 置き場 の対応表 -------------------------------------------
# YOLOv8 の既定モデルは COCO データセットの 80 クラスを出力する。
# そのうち「乗り物」を置き場A、「人とその持ち物」を置き場B に割り当てた。
# 表に無いクラスは「未知」として、あえて掴まずに待機させる。
# 判断できないものを無理に動かさないのは、実ロボットでは安全上とても大事。
BIN_BY_CLASS = {
    # 乗り物 -> 置き場A
    'bus': 'A', 'car': 'A', 'truck': 'A', 'train': 'A',
    'bicycle': 'A', 'motorcycle': 'A', 'airplane': 'A', 'boat': 'A',
    # 人とその持ち物 -> 置き場B
    'person': 'B', 'tie': 'B', 'backpack': 'B', 'handbag': 'B',
    'suitcase': 'B', 'umbrella': 'B', 'cell phone': 'B',
}


class SortingCommander(Node):
    """YOLO の判断を CRANE+ V2 の関節軌道に変換してパブリッシュするノード."""

    def __init__(self):
        super().__init__('sorting_commander')

        # --- パラメータ（ros2 run ... --ros-args -p 名前:=値 で外から変更できる）---
        home = os.path.expanduser('~/pai_final')
        self.declare_parameter('image', os.path.join(home, 'images/bus.jpg'))
        self.declare_parameter('model', os.path.join(home, 'yolov8n.pt'))
        self.declare_parameter('output_dir', os.path.join(home, 'output'))
        self.declare_parameter('r_pick', 0.15)      # 掴む位置の水平距離 [m]
        self.declare_parameter('z_hover', 0.060)    # 上空で待機する高さ [m]
        self.declare_parameter('z_grasp', 0.020)    # 掴む高さ [m]
        self.declare_parameter('pitch_deg', -90.0)  # 手先の向き（-90 = 真下）
        self.declare_parameter('bin_a_deg', 55.0)   # 置き場A の旋回角 [deg]
        self.declare_parameter('bin_b_deg', -55.0)  # 置き場B の旋回角 [deg]
        # グリッパの符号の向きは実機/シミュレータで実測して決めた。
        # crane_plus_joint_hand は【正で閉じ、負で開く】（直感と逆なので注意）。
        self.declare_parameter('gripper_open', -0.50)   # 負 = 開く
        self.declare_parameter('gripper_close', 0.55)   # 正 = 閉じる（実測で決めた把持値）
        self.declare_parameter('dry_run', False)    # True なら計算だけして動かさない
        # 退避姿勢（腕を折りたたんだ起立状態）。動作の前後でここへ戻す。
        self.declare_parameter('home_joint', [0.0, -1.16, -2.01, -0.73])

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.image = g('image')
        self.model = g('model')
        self.output_dir = g('output_dir')
        self.r_pick = g('r_pick')
        self.z_hover = g('z_hover')
        self.z_grasp = g('z_grasp')
        self.pitch = radians(g('pitch_deg'))
        self.bin_deg = {'A': g('bin_a_deg'), 'B': g('bin_b_deg')}
        self.gripper_open = g('gripper_open')
        self.gripper_close = g('gripper_close')
        self.dry_run = g('dry_run')
        self.home_joint = list(g('home_joint'))

        # --- パブリッシャ ---
        # CRANE+ V2 の ros2_control は、この 2 つのトピックで
        # JointTrajectory（関節の目標角と、そこへ到達する時間）を受け取る。
        self.joint_names = [
            'crane_plus_joint1', 'crane_plus_joint2',
            'crane_plus_joint3', 'crane_plus_joint4']
        self.pub_arm = self.create_publisher(
            JointTrajectory, 'crane_plus_arm_controller/joint_trajectory', 10)
        self.pub_gripper = self.create_publisher(
            JointTrajectory, 'crane_plus_gripper_controller/joint_trajectory', 10)

    # ------------------------------------------------------------------
    # 低レベル: トピックへ投げる
    # ------------------------------------------------------------------
    def publish_joint(self, q, dt):
        """4 関節の目標角 q [rad] へ dt 秒かけて動かす指令を出す."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        # time_from_start = 「軌道の開始から何秒後にこの点を通るか」。
        # 点を 1 個だけ入れれば「dt 秒かけてそこまで動け」という意味になり、
        # 途中は コントローラ側が滑らかに補間してくれる。
        point.time_from_start = Duration(
            seconds=int(dt), nanoseconds=int((dt - int(dt)) * 1e9)).to_msg()
        msg.points = [point]
        self.pub_arm.publish(msg)

    def publish_gripper(self, value, dt):
        """グリッパの開き（1 関節）を dt 秒かけて value [rad] にする."""
        msg = JointTrajectory()
        msg.joint_names = ['crane_plus_joint_hand']
        point = JointTrajectoryPoint()
        point.positions = [float(value)]
        point.time_from_start = Duration(
            seconds=int(dt), nanoseconds=int((dt - int(dt)) * 1e9)).to_msg()
        msg.points = [point]
        self.pub_gripper.publish(msg)

    # ------------------------------------------------------------------
    # 中レベル: 「どこへ手先を置くか」を関節角に変換する
    # ------------------------------------------------------------------
    def joints_at(self, yaw_deg, z):
        """旋回角 yaw_deg [deg]、半径 r_pick、高さ z [m] に手先を置く関節角を返す.

        工夫: 逆運動学の q2,q3,q4 は「半径 r と高さ z」だけで決まり、
        旋回角 q1 には依存しない（腕を横から見た形は、どちらを向いても同じだから）。
        そこで q1=0 の断面で 1 回だけ解き、あとは q1 を差し替えるだけで済ませている。
        掴む位置と置く位置で r と z を同じにしてあるので、
        「届くかどうか」の確認も 1 回で済む。
        """
        solution = inverse_kinematics([self.r_pick, 0.0, z, self.pitch],
                                      elbow_up=True)
        if solution is None:
            raise RuntimeError(
                f'逆運動学の解なし: r={self.r_pick:.3f} z={z:.3f} '
                f'pitch={degrees(self.pitch):.0f}deg（腕が届きません）')
        solution[0] = radians(yaw_deg)

        ok = joint_in_range(solution)
        if not all(ok):
            bad = [i + 1 for i, v in enumerate(ok) if not v]
            raise RuntimeError(
                f'関節 {bad} が可動域外: '
                + ', '.join(f'{degrees(v):.1f}deg' for v in solution))
        return solution

    # ------------------------------------------------------------------
    # 高レベル: 仕分け動作そのもの
    # ------------------------------------------------------------------
    def move(self, label, q, dt):
        line = ('  ' + label.ljust(16)
                + ' joint=[' + ', '.join(f'{degrees(v):6.1f}' for v in q) + '] deg')
        self.get_logger().info(line)
        if not self.dry_run:
            self.publish_joint(q, dt)
            time.sleep(dt)

    def grip(self, label, value, dt):
        if not gripper_in_range(value):
            raise RuntimeError(f'グリッパ指令 {value:.3f} が可動域外')
        self.get_logger().info(f'  {label.ljust(16)} gripper={value:.2f}')
        if not self.dry_run:
            self.publish_gripper(value, dt)
            time.sleep(dt)

    def run(self):
        log = self.get_logger()

        # ---- 1. 認識 ----
        log.info('=' * 62)
        log.info(f'[1/3] 画像認識: {self.image}')
        info = detect_main_object(self.image, self.model,
                                  output_dir=self.output_dir)
        if info is None:
            log.warn('物体を検出できませんでした。動作を中止します。')
            return
        log.info(f"      検出一覧: {info['all']}")
        log.info(f"      主対象  : {info['name']} "
                 f"(信頼度 {info['conf']:.2f} / 画面占有率 {info['area']:.1%})")
        if info['annotated']:
            log.info(f"      描画結果: {info['annotated']}")

        # ---- 2. 判断 ----
        bin_id = BIN_BY_CLASS.get(info['name'])
        log.info('[2/3] 仕分け判断')
        if bin_id is None:
            log.warn(f"      '{info['name']}' は対応表にありません -> 未知。"
                     '安全のため掴まずに待機します。')
            self.move('退避姿勢', self.home_joint, 3.0)
            return
        yaw = self.bin_deg[bin_id]
        log.info(f"      '{info['name']}' -> 置き場{bin_id}（旋回角 {yaw:+.0f}deg）")

        # ---- 3. 動作 ----
        log.info('[3/3] 仕分け動作')
        pick_hover = self.joints_at(0.0, self.z_hover)
        pick_grasp = self.joints_at(0.0, self.z_grasp)
        place_hover = self.joints_at(yaw, self.z_hover)
        place_put = self.joints_at(yaw, self.z_grasp)

        self.move('退避姿勢から開始', self.home_joint, 2.5)
        self.grip('グリッパ開', self.gripper_open, 1.5)
        self.move('掴む位置の上空', pick_hover, 3.0)
        self.move('下降', pick_grasp, 2.0)
        self.grip('グリッパ閉（把持）', self.gripper_close, 1.5)
        self.move('持ち上げ', pick_hover, 2.0)
        self.move(f'置き場{bin_id}へ旋回', place_hover, 3.0)
        self.move('下降', place_put, 2.0)
        self.grip('グリッパ開（解放）', self.gripper_open, 1.5)
        self.move('退避', place_hover, 2.0)
        self.move('退避姿勢へ', self.home_joint, 3.0)
        log.info(f"完了: {info['name']} を置き場{bin_id}へ仕分けしました。")
        log.info('=' * 62)


def main(args=None):
    rclpy.init(args=args)
    node = SortingCommander()

    # rclpy.spin() は「受信処理を回し続ける」無限ループ。
    # 本体の run() は上から順に進む手続きなので、spin を別スレッドに逃がして
    # 両方を同時に走らせる。（publish するだけなら厳密には不要だが、
    # ログのトピック送出などが滞らないようにするため教科書どおりにしておく）
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # パブリッシャが購読側（コントローラ）と接続を確立するまで少し待つ。
    # ここを省くと最初の 1 通が誰にも届かずに消える。
    time.sleep(1.0)

    try:
        node.run()
    except RuntimeError as e:
        node.get_logger().error(str(e))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
