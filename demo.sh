#!/bin/bash
# 動画撮影用の通しデモ。
# 同じプログラムが、入力画像の中身だけで行き先を変えることを示す。
#   1枚目 bus.jpg    -> 'bus'    -> 置き場A（左 +55 度）
#   2枚目 zidane.jpg -> 'person' -> 置き場B（右 -55 度）
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"

for IMG in "$HOME/pai_final/images/bus.jpg" "$HOME/pai_final/images/zidane.jpg"; do
  echo
  echo "################################################################"
  echo "#  入力画像: $(basename "$IMG")"
  echo "################################################################"
  "$HERE/reset_and_run.sh" "$IMG"
  sleep 2
done
echo
echo "デモ終了: 同一のプログラムが、画像の認識結果だけで置き場を切り替えました。"
