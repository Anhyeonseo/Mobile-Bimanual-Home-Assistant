#!/usr/bin/env python3
"""On-site check: is the ArUco marker detectable and large enough in saved colour images?

Reads image files only. No camera or robot access.
Usage: python3 tools/diagnostics/check_aruco_capture.py <color.png> [more images or folders ...]
"""
import sys
from pathlib import Path

import cv2
import numpy as np

MIN_SIDE_PX = 40  # below this, corner noise makes the pose unreliable


def detect(gray):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "ArucoDetector"):
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters()).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    return corners, ([] if ids is None else ids.flatten().tolist())


def images(arguments):
    for argument in arguments:
        path = Path(argument)
        if path.is_dir():
            yield from sorted(path.rglob("color.png"))
        else:
            yield path


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    bad = 0
    for path in images(sys.argv[1:]):
        image = cv2.imread(str(path))
        if image is None:
            print(f"READ FAIL  {path}")
            bad += 1
            continue
        height, width = image.shape[:2]
        corners, ids = detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        if not ids:
            print(f"NOT FOUND  {path}  ({width}x{height})")
            bad += 1
            continue
        for quad, marker_id in zip(corners, ids):
            points = quad.reshape(4, 2)
            sides = np.linalg.norm(points - np.roll(points, -1, axis=0), axis=1)
            centre = points.mean(axis=0)
            status = "OK       " if sides.min() >= MIN_SIDE_PX else "TOO SMALL"
            bad += status != "OK       "
            print(f"{status}  {path}  id={marker_id}  side={sides.min():.0f}-{sides.max():.0f}px  "
                  f"centre=({centre[0]:.0f},{centre[1]:.0f}) of {width}x{height}")
        cv2.aruco.drawDetectedMarkers(image, corners, np.asarray(ids).reshape(-1, 1))
        cv2.imwrite(str(path.with_name(path.stem + "_aruco_check.jpg")), image)
    print(f"\n{'ALL OK' if not bad else f'{bad} problem(s)'}")


if __name__ == "__main__":
    main()
