"""Printable arm markers for the calibration tour: one ArUco id per arm, A4 portrait.

  left arm   ArUco DICT_4X4_50 id 4, 70.0 mm black square   (same id as the 2026-11-13 capture)
  right arm  ArUco DICT_4X4_50 id 5, 70.0 mm black square
Two copies of each (one spare). Every marker has a 10 mm white quiet zone inside a dashed cut line
(90 x 90 mm). A 100 mm scale bar checks the print scale. calib_model.py reads MARKER_ID.

usage: python3 make_arm_markers.py OUT.pdf
"""
import sys
import cv2
from PIL import Image, ImageDraw, ImageFont

DPI = 600
MM = DPI / 25.4
SIDE_MM, QUIET_MM = 70.0, 10.0
W, H = round(210 * MM), round(297 * MM)


def px(mm): return int(round(mm * MM))


img = Image.new('L', (W, H), 255); draw = ImageDraw.Draw(img)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try:
    font = ImageFont.truetype('DejaVuSans.ttf', px(3.5)); big = ImageFont.truetype('DejaVuSans-Bold.ttf', px(5))
except OSError:
    font = big = ImageFont.load_default()

side, cell = px(SIDE_MM), px(SIDE_MM + 2 * QUIET_MM)
x0, y0, gap = (W - 2 * cell - px(10)) // 2, px(22), px(10)
layout = [(4, 'LEFT arm  id 4'), (4, 'LEFT spare  id 4'), (5, 'RIGHT arm  id 5'), (5, 'RIGHT spare  id 5')]
for k, (marker_id, label) in enumerate(layout):
    col, row = k % 2, k // 2
    cx, cy = x0 + col * (cell + gap), y0 + row * (cell + gap + px(8))
    for i in range(0, cell, px(4)):                                   # dashed cut line
        for a, b in (((cx + i, cy), (cx + min(i + px(2), cell), cy)), ((cx + i, cy + cell), (cx + min(i + px(2), cell), cy + cell)),
                     ((cx, cy + i), (cx, cy + min(i + px(2), cell))), ((cx + cell, cy + i), (cx + cell, cy + min(i + px(2), cell)))):
            draw.line([a, b], fill=0, width=px(0.2))
    m = cv2.aruco.generateImageMarker(dictionary, marker_id, side)
    img.paste(Image.fromarray(m), (cx + px(QUIET_MM), cy + px(QUIET_MM)))
    draw.text((cx, cy + cell + px(1.5)), label, fill=0, font=big)

y = y0 + 2 * (cell + gap + px(8)) + px(8)
cx = W // 2
draw.line([(cx - px(50), y), (cx + px(50), y)], fill=0, width=px(0.4))
for x in range(-50, 51, 10):
    draw.line([(cx + px(x), y - px(2 if x % 50 else 4)), (cx + px(x), y)], fill=0, width=px(0.3))
lines = ['100 mm - check with a ruler; print at 100 % (no fit-to-page)',
         'ArUco 4x4_50, black square 70.0 mm. Cut on the dashed line (90 x 90 mm), keep the white border.',
         'Measure all four black edges with calipers (0.1 mm) and write them down.']
for i, t in enumerate(lines):
    draw.text((cx - px(80), y + px(3 + 6 * i)), t, fill=0, font=font)
img.save(sys.argv[1], resolution=DPI)
print('wrote', sys.argv[1], f'{W}x{H} px at {DPI} dpi')
