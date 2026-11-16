"""Printable lens-check checkerboard, A4 landscape: 10 x 7 squares of 25.0 mm (9 x 6 inner corners).

intrinsics_check.py finds the 9 x 6 inner-corner pattern. Glue the print flat on a rigid board and
measure the real square size (five squares with a ruler, divided by five) before use.

usage: python3 make_checkerboard.py OUT.pdf
"""
import sys
from PIL import Image, ImageDraw, ImageFont

DPI = 600
MM = DPI / 25.4
SQ_MM, NX, NY = 25.0, 10, 7
W, H = round(297 * MM), round(210 * MM)


def px(mm): return int(round(mm * MM))


img = Image.new('L', (W, H), 255); draw = ImageDraw.Draw(img)
x0, y0 = (W - px(SQ_MM * NX)) // 2, px(12)
for i in range(NX):
    for j in range(NY):
        if (i + j) % 2 == 0:
            draw.rectangle([x0 + px(i * SQ_MM), y0 + px(j * SQ_MM), x0 + px((i + 1) * SQ_MM) - 1, y0 + px((j + 1) * SQ_MM) - 1], fill=0)
try:
    font = ImageFont.truetype('DejaVuSans.ttf', px(3.2))
except OSError:
    font = ImageFont.load_default()
y = y0 + px(SQ_MM * NY) + px(8)
draw.text((x0, y), f'{NX} x {NY} squares, {SQ_MM:.1f} mm ({NX - 1} x {NY - 1} inner corners). Print at 100 % (no fit-to-page). '
          'Measure 5 squares with a ruler and divide by 5.', fill=0, font=font)
img.save(sys.argv[1], resolution=DPI)
print('wrote', sys.argv[1])
