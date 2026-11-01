"""YOLOv8 による物体認識パート.

ROS に依存しない純粋な関数として切り出してある。
こうしておくと Gazebo を起動していなくても認識だけ単体で試せるし、
「認識」と「ロボット制御」のどちらが原因でおかしいのかを切り分けやすい。
"""

import os

# ultralytics は import しただけで重いので、関数の中で遅延 import する。
# （ros2 run で起動したときに import エラーの場所が分かりやすくなる副次効果もある）


def detect_main_object(image_path, model_path, conf_threshold=0.25,
                       output_dir=None):
    """画像 1 枚を YOLOv8 にかけ、「主対象」1 個の情報を返す.

    引数:
        image_path   : 入力画像のパス
        model_path   : 学習済み重み（yolov8n.pt）のパス
        conf_threshold: この信頼度未満の検出は捨てる
        output_dir   : 認識結果を描き込んだ画像の保存先（None なら保存しない）

    戻り値: dict
        {'name': クラス名, 'conf': 信頼度, 'area': 面積の画像比,
         'all': [(クラス名, 信頼度), ...], 'annotated': 保存した画像パス or None}
        何も検出できなければ None

    「主対象」の決め方:
        信頼度が一番高いものではなく、**バウンディングボックスの面積が一番大きい**
        ものを選ぶ。理由は、仕分けの対象はカメラの手前にある物体であり、
        画面上では大きく写るから。信頼度最大だと、背景に小さく写っている
        別のものが選ばれてしまうことがある。
    """
    from ultralytics import YOLO

    model = YOLO(model_path)
    # verbose=False にしないと 1 枚ごとに大量のログが出て、動画が見づらくなる
    results = model(image_path, conf=conf_threshold, verbose=False)
    result = results[0]

    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return None

    # xyxy は [左, 上, 右, 下] のピクセル座標。面積 = 幅 x 高さ。
    xyxy = boxes.xyxy.cpu().numpy()
    widths = xyxy[:, 2] - xyxy[:, 0]
    heights = xyxy[:, 3] - xyxy[:, 1]
    areas = widths * heights

    best = int(areas.argmax())

    class_ids = boxes.cls.cpu().numpy().astype(int)
    confs = boxes.conf.cpu().numpy()
    names = result.names  # {クラス番号: クラス名} の辞書

    img_h, img_w = result.orig_shape
    annotated_path = None
    if output_dir is not None:
        import cv2
        os.makedirs(output_dir, exist_ok=True)
        # plot() は検出枠とラベルを描き込んだ画像(BGR の numpy 配列)を返す
        annotated = result.plot()
        base = os.path.splitext(os.path.basename(image_path))[0]
        annotated_path = os.path.join(output_dir, base + '_detected.jpg')
        cv2.imwrite(annotated_path, annotated)

    return {
        'name': names[class_ids[best]],
        'conf': float(confs[best]),
        'area': float(areas[best]) / float(img_w * img_h),
        'all': [(names[c], float(p)) for c, p in zip(class_ids, confs)],
        'annotated': annotated_path,
    }


def main():
    """認識だけを単体で確認するコマンド: ros2 run pai_final_sorter vision_test <画像>."""
    import sys
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if not args:
        print('使い方: ros2 run pai_final_sorter vision_test <画像パス> [重みパス]')
        return
    image = args[0]
    model = args[1] if len(args) > 1 else os.path.expanduser('~/pai_final/yolov8n.pt')
    info = detect_main_object(image, model, output_dir=os.path.dirname(image) or '.')
    if info is None:
        print('検出なし')
        return
    print(f"主対象: {info['name']} (信頼度 {info['conf']:.2f}, 面積比 {info['area']:.3f})")
    print('全検出:', info['all'])
    print('描画結果:', info['annotated'])
