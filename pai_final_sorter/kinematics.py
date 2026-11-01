"""CRANE+ V2 の運動学（順運動学・逆運動学）.

このファイルは外部パッケージに依存せず、math だけで完結している。
単位はすべて SI（長さ: m、角度: rad）。

CRANE+ V2 は「土台の旋回 + 肩 + 肘 + 手首」の 4 自由度アーム。
4 自由度では手先の位置(x, y, z)と姿勢(roll, pitch, yaw)の 6 個すべてを
自由に決めることはできない。そこで本実装では
  - yaw は土台の旋回角 q1 で自動的に決まる（手先は常に腕の伸びる方向を向く）
  - roll は機構上ゼロ固定
  - 指定できるのは (x, y, z, pitch) の 4 個
という「4 入力 4 関節」の形に落としている。これが解ける理由。
"""

from math import acos, atan2, cos, pi, sin, sqrt

# --- リンク長 [m] -----------------------------------------------------------
# 土台の天面から第2関節（肩）までの高さ
L1 = 0.0456 + 0.026
# 肩 → 肘
L2 = 0.083
# 肘 → 手首
L3 = 0.0935
# 手首 → 手先（グリッパの先端）
L4 = 0.045 + 0.045

RAD = pi / 180.0

# --- 関節可動域 [rad] -------------------------------------------------------
# 実機 CRANE+ V2 のサーボ可動域。ここを外れる指令を出すとサーボが唸る／壊れる。
JOINT_MIN = [-150.00 * RAD, -104.55 * RAD, -146.48 * RAD, -105.43 * RAD]
JOINT_MAX = [150.00 * RAD, 102.20 * RAD, 140.62 * RAD, 101.32 * RAD]

# グリッパ（1 自由度）。両端ちょうどだと丸め誤差で範囲外になるので 0.001 内側に寄せる。
GRIPPER_MIN = -40.62 * RAD + 0.001
GRIPPER_MAX = 38.27 * RAD - 0.001


def joint_in_range(joint):
    """4 関節それぞれが可動域に入っているかを True/False のリストで返す."""
    return [lo <= q <= hi for q, lo, hi in zip(joint, JOINT_MIN, JOINT_MAX)]


def gripper_in_range(gripper):
    """グリッパ指令が可動域内かどうか."""
    return GRIPPER_MIN <= gripper <= GRIPPER_MAX


def normalize_angle(x):
    """角度を -pi 〜 +pi の範囲に畳み込む.

    atan2 や acos の結果を足し引きすると pi を超えることがある。
    「+350 度」と「-10 度」は同じ姿勢なので、後者の表現に揃えておかないと
    可動域チェックで不必要に「範囲外」と判定されてしまう。
    """
    while x < -pi:
        x += 2 * pi
    while x > pi:
        x -= 2 * pi
    return x


def forward_kinematics(joint):
    """関節角 -> 手先位置姿勢. joint=[q1,q2,q3,q4] -> [x, y, z, pitch].

    考え方:
      1. まず腕を真横から見た 2 次元平面（半径 r と高さ z の平面）で考える。
         肩・肘・手首の 3 つの角度を足し合わせながらリンクを順に伸ばしていくと
         手先の (r, z) が出る。
      2. その平面全体を土台の旋回角 q1 だけ回すと、3 次元の (x, y) になる。
    """
    q1, q2, q3, q4 = joint
    # CRANE+ の仕様では q2 は「鉛直からの傾き」かつ回転向きが逆なので、
    # 数式を素直に書けるよう「水平からの仰角」に変換する。
    q2 = pi / 2 - q2

    r = L2 * cos(q2) + L3 * cos(q2 + q3) + L4 * cos(q2 + q3 + q4)
    z = L1 + L2 * sin(q2) + L3 * sin(q2 + q3) + L4 * sin(q2 + q3 + q4)

    x = r * cos(q1)
    y = r * sin(q1)
    pitch = q2 + q3 + q4
    return [x, y, z, pitch]


def inverse_kinematics(endtip, elbow_up=True):
    """手先位置姿勢 -> 関節角. endtip=[x, y, z, pitch] -> [q1,q2,q3,q4] or None.

    解き方（幾何的に閉形式で解く。数値反復は使わない）:

      手順1: 土台の旋回角 q1 は、真上から見た方向そのもの。q1 = atan2(y, x)。
             これで 3 次元問題が「半径 r と高さ z の 2 次元問題」に落ちる。

      手順2: 手先の姿勢 pitch は指定値なので、最後のリンク L4 が向く方向は確定している。
             そこで手先から L4 を逆向きにたどって「手首の位置 (rr, zz)」を先に求める。
             こうすると残るのは L2 と L3 の 2 リンクだけになる。

      手順3: 肩から手首までの直線距離 d が分かれば、三角形の 3 辺
             (L2, L3, d) が揃う。余弦定理で肘の角度 q3 が一意に出る。
             |cos(q3)| > 1 なら三角形が作れない = その点は腕が届かない（解なし）。

      手順4: 肩の角度 q2 は「肩から手首を見上げる角 phi」と
             「その直線から L2 がどれだけずれるか psi」の和（または差）。
             和を取るのが elbow_up=True（肘が上に張り出す姿勢）、
             差を取るのが elbow_up=False（肘が下に落ちる姿勢）。
             同じ手先位置に 2 通りの肘の曲げ方があるので、どちらかを選ぶ必要がある。

      手順5: pitch = q2 + q3 + q4 なので、q4 は引き算で求まる。
    """
    x, y, z, pitch = endtip

    r = sqrt(x ** 2 + y ** 2)
    if r == 0.0:
        return None  # 真上は旋回角が定まらないので除外

    q1 = atan2(y, x)

    # 手順2: 手先から L4 を逆にたどって手首位置を出す
    rr = r - L4 * cos(pitch)
    zz = z - L4 * sin(pitch)

    # 手順3: 肩(高さ L1)から手首までの距離
    d = sqrt(rr ** 2 + (zz - L1) ** 2)
    cq3 = (d ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2 * L3)
    if abs(cq3) > 1.0:
        return None  # 届かない

    # 手順4
    phi = atan2(zz - L1, rr)
    psi = acos((d ** 2 + L2 ** 2 - L3 ** 2) / (2 * L2 * d))
    if elbow_up:
        q2 = normalize_angle(phi + psi)
        q3 = -acos(cq3)
    else:
        q2 = normalize_angle(phi - psi)
        q3 = acos(cq3)

    # 手順5
    q4 = normalize_angle(pitch - q2 - q3)

    # 計算用の「水平からの仰角」を CRANE+ の仕様（鉛直基準・逆向き）に戻す
    q2 = pi / 2 - q2
    return [q1, q2, q3, q4]
