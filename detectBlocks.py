"""
detectBlocks.py

Open a USB camera with OpenCV, find Mega Bloks in red, yellow, green,
and blue, and return each detection's image position, width, and color.

Position is the contour centroid in pixels (origin at the top-left of
the frame). Width is the shorter side of the oriented bounding box, in
pixels — the dimension the gripper would typically close on.

Fill in PIXEL → MM CONVERSION below, then call get_one_block() to get a
single target for pickPlace.

For a live demo the camera should stay open: use BlockStream. It keeps
frames flowing, get_one_block() samples the latest frame, and the preview
window stays up while the arm runs:

    python3 pickPlace.py --demo
    python3 pickPlace.py --demo --color red

Usage:
    python3 detectBlocks.py
    python3 detectBlocks.py --camera 1

Press q in the preview window to quit.

Author: Paul Cairns
Date: Oct 6th, 2026
"""
from __future__ import annotations

import argparse
import glob
import math
import sys
import threading
import time

import cv2
import numpy as np

# HSV ranges are (lower, upper) in OpenCV scale: H 0-179, S 0-255, V 0-255.
# Red wraps around hue 0, so it uses two ranges.
COLOR_RANGES = {
    "red": [
        (np.array([0, 120, 70]), np.array([10, 255, 255])),
        (np.array([170, 120, 70]), np.array([179, 255, 255])),
    ],
    "yellow": [
        (np.array([18, 100, 80]), np.array([35, 255, 255])),
    ],
    "green": [
        (np.array([40, 70, 50]), np.array([85, 255, 255])),
    ],
    "blue": [
        (np.array([110, 50, 40]), np.array([130, 255, 255])),
    ],
}

DRAW_COLORS = {
    "red": (0, 0, 255),
    "yellow": (0, 255, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
}

MIN_AREA_PX = 800
MAX_ASPECT = 3.5
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

# ---------------------------------------------------------------------------
# PIXEL → MM CONVERSION  (your camera-to-robot coordinate system)
# ---------------------------------------------------------------------------
# Image: origin at top-left, +u right, +v down, units = pixels.
# Robot: tool-frame millimetres used by pickPlace / inverse kinematics.
#
# Measure one known landmark that you can see in the image and in robot
# millimetres, then fill in the values below.
#
#   1. IMAGE_ORIGIN_U/V  = pixel (u, v) of that landmark
#   2. ROBOT_ORIGIN_X/Y  = robot (x, y) mm of that same landmark
#   3. MM_PER_PIXEL_U/V  = millimetres per pixel along each image axis
#                          (use a ruler or a known Mega Blok size)
#   4. CAMERA_YAW_DEG    = rotation from image axes to robot XY (degrees)
#   5. TABLE_Z_MM        = pick height of a block sitting on the table
#
# Linear model used by pixels_to_mm():
#   du = (u - IMAGE_ORIGIN_U) * MM_PER_PIXEL_U
#   dv = (v - IMAGE_ORIGIN_V) * MM_PER_PIXEL_V
#   x  = ROBOT_ORIGIN_X_MM + du * cos(yaw) - dv * sin(yaw)
#   y  = ROBOT_ORIGIN_Y_MM + du * sin(yaw) + dv * cos(yaw)
#   z  = TABLE_Z_MM
#   width_mm = width_px * mean(|MM_PER_PIXEL_U|, |MM_PER_PIXEL_V|)
#
# Replace pixels_to_mm() entirely if you later use a homography,
# chessboard calibration, or a different axis mapping.

IMAGE_ORIGIN_U = 640.0
IMAGE_ORIGIN_V = 360.0
MM_PER_PIXEL_U = 1.0
MM_PER_PIXEL_V = 1.0
CAMERA_YAW_DEG = 0.0
ROBOT_ORIGIN_X_MM = 0.0
ROBOT_ORIGIN_Y_MM = 0.0
TABLE_Z_MM = 35.0


def pixels_to_mm(u, v, width_px):
    """
    Convert one image detection into robot millimetres.

    Returns (x_mm, y_mm, z_mm, width_mm). Edit this function (or the
    constants above) to match your camera mount and coordinate system.
    """
    du = (float(u) - IMAGE_ORIGIN_U) * MM_PER_PIXEL_U
    dv = (float(v) - IMAGE_ORIGIN_V) * MM_PER_PIXEL_V
    yaw = math.radians(CAMERA_YAW_DEG)
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    x_mm = ROBOT_ORIGIN_X_MM + du * cos_y - dv * sin_y
    y_mm = ROBOT_ORIGIN_Y_MM + du * sin_y + dv * cos_y
    z_mm = TABLE_Z_MM
    mm_per_px = 0.5 * (abs(MM_PER_PIXEL_U) + abs(MM_PER_PIXEL_V))
    width_mm = float(width_px) * mm_per_px
    return x_mm, y_mm, z_mm, width_mm


def as_pick_target(detection):
    """
    Turn one pixel detection into a pickPlace-ready dict.

    xyz    [x, y, z] millimetres for pick_place_sequence()
    width  millimetres for the gripper close_width
    """
    u, v = detection["position"]
    x_mm, y_mm, z_mm, width_mm = pixels_to_mm(u, v, detection["width"])
    return {
        "color": detection["color"],
        "xyz": [x_mm, y_mm, z_mm],
        "width": width_mm,
        "position_px": (u, v),
        "width_px": detection["width"],
        "length_px": detection["length"],
        "angle": detection["angle"],
    }


def choose_block(detections, color=None, index=0):
    """
    Pick one detection from a list.

    color=None  → any color, already sorted largest-first
    color="red" → only that color (largest first)
    index       → 0 is the first match, 1 the next, and so on
    """
    if color is not None:
        color = color.lower()
        if color not in COLOR_RANGES:
            raise ValueError(f"Unknown color {color!r}. Use one of {list(COLOR_RANGES)}.")
        detections = [d for d in detections if d["color"] == color]
    if index < 0 or index >= len(detections):
        return None
    return detections[index]


def get_one_block(color=None, index=0, camera_index=None, frame=None, stream=None):
    """
    Return one Mega Blok in robot millimetres, or None if none match.

    Intended for an agent that pick-places one block at a time:

        target = get_one_block(color="blue")
        if target is None:
            ...
        pick_place_sequence(target["xyz"], place_xyz, close_width=target["width"])

    Pass a BlockStream to sample the live camera without closing it.
    Pass an existing BGR frame to skip opening the camera.
    """
    if stream is not None:
        return stream.get_one_block(color=color, index=index)
    if frame is None:
        detections, frame = capture_and_detect(camera_index)
    else:
        detections = detect_blocks(frame)
    chosen = choose_block(detections, color=color, index=index)
    if chosen is None:
        return None
    return as_pick_target(chosen)


class BlockStream:
    """
    Keep the USB camera streaming in the background.

    The agent calls get_one_block() whenever it needs one target. The
    preview window stays open so you can watch the arm during pick-place.
    OpenCV display must run on the main thread: call show() and leave it
    there; run the robot on another thread.
    """

    def __init__(self, camera_index=None, window="Mega Blok detection"):
        self.cap = open_camera(camera_index)
        self.window = window
        self.lock = threading.Lock()
        self.frame = None
        self.detections = []
        self.highlight = None
        self.status = ""
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if self._thread is not None:
            return self
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        return self

    def _capture_loop(self):
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            detections = detect_blocks(frame)
            with self.lock:
                self.frame = frame
                self.detections = detections

    def wait_for_frame(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.lock:
                if self.frame is not None:
                    return True
            time.sleep(0.05)
        return False

    def set_status(self, text):
        with self.lock:
            self.status = text
        print(text)

    def get_one_block(self, color=None, index=0):
        """Sample the latest live frame. Does not stop the stream."""
        if not self.wait_for_frame():
            return None
        with self.lock:
            detections = list(self.detections)
        chosen = choose_block(detections, color=color, index=index)
        if chosen is None:
            return None
        target = as_pick_target(chosen)
        with self.lock:
            self.highlight = chosen
        return target

    def _preview_frame(self):
        with self.lock:
            if self.frame is None:
                return None
            frame = self.frame.copy()
            detections = list(self.detections)
            highlight = self.highlight
            status = self.status
        preview = draw_detections(frame, detections, highlight=highlight)
        if status:
            cv2.putText(
                preview,
                status,
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        return preview

    def show(self):
        """Run the preview window on the main thread until q or window close."""
        print("Camera streaming. Press q (or close the window) to quit.")
        shown = False
        try:
            while not self._stop.is_set():
                preview = self._preview_frame()
                if preview is not None:
                    cv2.imshow(self.window, preview)
                    shown = True
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if shown and cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1:
                    break
        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            self.stop()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        cv2.destroyAllWindows()


def list_camera_indices():
    """Return /dev/videoN numbers that currently exist (Linux)."""
    indices = []
    for path in sorted(glob.glob("/dev/video*")):
        suffix = path.replace("/dev/video", "")
        if suffix.isdigit():
            indices.append(int(suffix))
    return indices


def _try_open_index(index, width, height):
    backends = []
    if hasattr(cv2, "CAP_V4L2"):
        backends.append(cv2.CAP_V4L2)
    backends.append(cv2.CAP_ANY)

    for source in (index, f"/dev/video{index}"):
        for backend in backends:
            cap = cv2.VideoCapture(source, backend)
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ok, _ = cap.read()
            if ok:
                return cap
            cap.release()
    return None


def open_camera(index=None, width=1280, height=720):
    """
    Open a USB camera that can actually deliver frames.

    If index is None, try every /dev/videoN (USB cameras often appear as
    video1/video2 after unplug, not video0). Metadata-only nodes are skipped.
    """
    available = list_camera_indices()
    if index is None:
        candidates = available if available else [0, 1, 2]
    else:
        candidates = [index]

    for candidate in candidates:
        cap = _try_open_index(candidate, width, height)
        if cap is not None:
            print(f"Opened camera index {candidate} (/dev/video{candidate}).")
            return cap

    raise RuntimeError(
        "Could not open a camera. Available devices: "
        f"{available or 'none'}. Close any leftover python3 detectBlocks.py "
        "with Ctrl+C, then retry, e.g. python3 detectBlocks.py --camera 1"
    )


def _mask_for_color(hsv, color):
    mask = None
    for lower, upper in COLOR_RANGES[color]:
        part = cv2.inRange(hsv, lower, upper)
        mask = part if mask is None else cv2.bitwise_or(mask, part)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=2)
    return mask


def _contour_to_detection(contour, color):
    area = float(cv2.contourArea(contour))
    if area < MIN_AREA_PX:
        return None

    rect = cv2.minAreaRect(contour)
    (cx, cy), (w, h), angle = rect
    if w <= 0 or h <= 0:
        return None

    long_side = max(w, h)
    short_side = min(w, h)
    aspect = long_side / short_side
    if aspect > MAX_ASPECT:
        return None

    return {
        "color": color,
        "position": (float(cx), float(cy)),
        "width": float(short_side),
        "length": float(long_side),
        "angle": float(angle),
        "area": area,
        "box": cv2.boxPoints(rect),
    }


def detect_blocks(frame):
    """
    Find Mega Bloks in a BGR frame.

    Returns a list of dicts:
        color     str
        position  (x, y) centroid in pixels
        width     shorter side of the oriented box, pixels
        length    longer side of the oriented box, pixels
        angle     OpenCV minAreaRect angle, degrees
        area      contour area, pixels
        box       4x2 ndarray of box corners
    """
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    detections = []
    for color in COLOR_RANGES:
        mask = _mask_for_color(hsv, color)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            detection = _contour_to_detection(contour, color)
            if detection is not None:
                detections.append(detection)

    detections.sort(key=lambda d: d["area"], reverse=True)
    return detections


def detect_block(frame, color):
    """Return the largest pixel detection of the given color, or None."""
    return choose_block(detect_blocks(frame), color=color, index=0)


def draw_detections(frame, detections, highlight=None):
    """Draw labeled boxes on a copy of the frame."""
    out = frame.copy()
    highlight_pos = None if highlight is None else highlight["position"]
    for d in detections:
        box = np.int32(d["box"])
        bgr = DRAW_COLORS[d["color"]]
        is_chosen = highlight_pos is not None and d["position"] == highlight_pos
        cv2.drawContours(out, [box], 0, (255, 255, 255) if is_chosen else bgr, 4 if is_chosen else 2)
        x, y = d["position"]
        cv2.circle(out, (int(x), int(y)), 4, bgr, -1)
        x_mm, y_mm, z_mm, w_mm = pixels_to_mm(x, y, d["width"])
        label = (
            f"{d['color']}  px=({x:.0f},{y:.0f})  "
            f"mm=({x_mm:.0f},{y_mm:.0f},{z_mm:.0f})  w={w_mm:.0f}mm"
        )
        if is_chosen:
            label = "PICK  " + label
        cv2.putText(
            out,
            label,
            (int(x) - 80, int(y) - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            bgr,
            2,
            cv2.LINE_AA,
        )
    if highlight is not None and highlight_pos is not None:
        hx, hy = highlight_pos
        if not any(d["position"] == highlight_pos for d in detections):
            cv2.circle(out, (int(hx), int(hy)), 8, (255, 255, 255), 2)
    return out


def capture_and_detect(camera_index=None):
    """Grab one frame from the USB camera and return detections."""
    cap = open_camera(camera_index)
    try:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("Camera opened but failed to read a frame.")
        return detect_blocks(frame), frame
    finally:
        cap.release()


def main():
    parser = argparse.ArgumentParser(description="Detect colored Mega Bloks from a USB camera.")
    parser.add_argument(
        "--camera",
        type=int,
        default=None,
        help="Video capture index. Default: first /dev/videoN that returns frames.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Read one frame, print detections, and exit (no preview window).",
    )
    parser.add_argument(
        "--color",
        default=None,
        help="With --once, return only this color (red, yellow, green, blue).",
    )
    parser.add_argument(
        "--index",
        type=int,
        default=0,
        help="With --once, which matching block to return (0 = largest).",
    )
    args = parser.parse_args()

    if args.once:
        detections, frame = capture_and_detect(args.camera)
        if not detections:
            print("No Mega Bloks detected.")
            return 0
        print("All detections (pixels):")
        for d in detections:
            x, y = d["position"]
            print(
                f"  color={d['color']}  position=({x:.1f}, {y:.1f})  "
                f"width={d['width']:.1f}px  length={d['length']:.1f}px"
            )
        target = get_one_block(color=args.color, index=args.index, frame=frame)
        if target is None:
            print(f"No block matched color={args.color!r} index={args.index}.")
            return 0
        print("Chosen block for pickPlace:")
        print(
            f"  color={target['color']}  xyz_mm={target['xyz']}  "
            f"width_mm={target['width']:.1f}"
        )
        return 0

    stream = BlockStream(args.camera).start()
    stream.set_status("Streaming. Press q to quit.")
    stream.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
