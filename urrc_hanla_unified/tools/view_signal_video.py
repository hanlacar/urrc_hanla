#!/usr/bin/env python3
"""View the existing finish-signal HSV classifier on a video without ROS."""
import argparse
from collections import Counter
from pathlib import Path
import sys
import time

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/race_perception"))
from race_perception.traffic_light_color import finish_signal_from_bgr


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video")
    parser.add_argument("--roi", type=float, nargs=4, default=[0.2, 0.05, 0.8, 0.70],
                        metavar=("X1", "Y1", "X2", "Y2"))
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args()
    x1, y1, x2, y2 = args.roi
    if not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
        parser.error("ROI must satisfy 0 <= X1 < X2 <= 1 and 0 <= Y1 < Y2 <= 1")
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        parser.error(f"Cannot open video: {args.video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    counts = Counter()
    window = "OpenCV signal: q=quit, space=pause, r=select ROI"
    print("Raw HSV RED/GREEN classifier; UNKNOWN includes unsupported colors.", flush=True)
    print("Press r and drag around the lamp to exclude colored background objects.", flush=True)
    try:
        while True:
            start = time.monotonic()
            ok, frame = cap.read()
            if not ok:
                break
            # Match the existing ROS video input and its pixel-area thresholds.
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)
            state, detail = finish_signal_from_bgr(frame, args.roi)
            counts[state] += 1
            if not args.headless:
                display = frame.copy()
                left, top, right, bottom = detail["roi_xyxy"]
                color = {"RED": (0, 0, 255), "GREEN": (0, 255, 0)}.get(state, (0, 255, 255))
                cv2.rectangle(display, (left, top), (right, bottom), color, 2)
                label = (f"HSV: {state}  red={detail['red_area']} "
                         f"green={detail['green_area']}")
                cv2.rectangle(display, (0, 0), (640, 30), (0, 0, 0), -1)
                cv2.putText(display, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
                cv2.imshow(window, display)
                delay = max(1, int(1000 * (1 / fps - (time.monotonic() - start))))
                key = cv2.waitKey(delay) & 0xFF
                if key == ord(" "):
                    key = cv2.waitKey(0) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("r"):
                    x, y, w, h = cv2.selectROI("Select lamp ROI", frame, False, False)
                    cv2.destroyWindow("Select lamp ROI")
                    if w and h:
                        args.roi = [x / 640, y / 480, (x + w) / 640, (y + h) / 480]
                        print(f"ROI: {args.roi}", flush=True)
            if args.max_frames > 0 and sum(counts.values()) >= args.max_frames:
                break
    finally:
        cap.release()
        if not args.headless:
            cv2.destroyAllWindows()
    print(f"Frame counts (not accuracy): {dict(counts)}", flush=True)


if __name__ == "__main__":
    main()
