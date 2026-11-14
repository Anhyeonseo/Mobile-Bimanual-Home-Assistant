"""Printable gripper-tip (TCP) touch target: a cross-hair between two ArUco markers, A4 landscape.

Layout (millimetres, marker frame x to the right, y up, origin at the cross-hair):
  ArUco DICT_4X4_50 id 1, 70.0 mm black square, centred at (-80, 0)
  ArUco DICT_4X4_50 id 2, 70.0 mm black square, centred at (+80, 0)
  cross-hair + 3 mm circle at (0, 0)          <- touch the closed gripper tip here
  100 mm scale bar under the markers          <- check the print scale with a ruler
Either marker alone gives the cross-hair position, so the gripper may hide one of them.
tcp_touch.py reads the same constants.

usage: python3 make_tcp_target.py OUT.pdf
"""
import sys
import numpy as np, cv2
from PIL import Image, ImageDraw, ImageFont

DPI = 600
MM = DPI / 25.4
SIDE_MM, OFFSET_MM = 70.0, 80.0
W, H = round(297 * MM), round(210 * MM)


def px(mm): return int(round(mm * MM))


img = Image.new('L', (W, H), 255); draw = ImageDraw.Draw(img)
cx, cy = W // 2, H // 2 - px(10)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
side = px(SIDE_MM)
for marker_id, dx in ((1, -OFFSET_MM), (2, OFFSET_MM)):
    m = cv2.aruco.generateImageMarker(dictionary, marker_id, side)
    img.paste(Image.fromarray(m), (cx + px(dx) - side // 2, cy - side // 2))
r = px(1.5)
draw.ellipse([cx - r, cy - r, cx + r, cy + r], outline=0, width=px(0.3))
for a, b in (((cx - px(12), cy), (cx - px(3), cy)), ((cx + px(3), cy), (cx + px(12), cy)),
             ((cx, cy - px(12)), (cx, cy - px(3))), ((cx, cy + px(3)), (cx, cy + px(12)))):
    draw.line([a, b], fill=0, width=px(0.3))
y = cy + side // 2 + px(18)
draw.line([(cx - px(50), y), (cx + px(50), y)], fill=0, width=px(0.4))
for x in range(-50, 51, 10):
    draw.line([(cx + px(x), y - px(2 if x % 50 else 4)), (cx + px(x), y)], fill=0, width=px(0.3))
try:
    font = ImageFont.truetype('DejaVuSans.ttf', px(3.5))
except OSError:
    font = ImageFont.load_default()
draw.text((cx - px(50), y + px(3)), '100 mm - check with a ruler; print at 100 % (no fit-to-page)', fill=0, font=font)
draw.text((cx - px(50), y + px(9)), 'TCP touch target  |  ArUco 4x4_50 id 1 (left) / id 2 (right), 70.0 mm, centres 80 mm from the cross-hair',
          fill=0, font=font)
img.save(sys.argv[1], resolution=DPI)
print('wrote', sys.argv[1], f'{W}x{H} px at {DPI} dpi')
