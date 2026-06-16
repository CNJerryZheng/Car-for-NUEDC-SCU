# Green target detection debug script for OpenMV IDE.

import csi
import time

# LAB threshold for green targets.
# Tune this value on-site with OpenMV IDE threshold editor.
GREEN_THRESHOLDS = (20, 90, -80, -10, -10, 80)

PIXELS_THRESHOLD = 250
AREA_THRESHOLD = 250
MIN_BLOB_W = 12
MIN_BLOB_H = 12
MAX_BLOB_W = 300
MAX_BLOB_H = 220
MAX_ASPECT_RATIO = 2.8
MIN_DENSITY = 0.45
STABLE_FRAMES = 2
MAX_CENTER_JUMP = 45

FRAME_WIDTH = 320
FRAME_CENTER_X = FRAME_WIDTH // 2

# Camera mounting calibration. Keep these values the same as green_uart_tx.py.
X_SIGN = 1
X_OFFSET = 0

# Put the car 40 mm from the green target, read the printed blob height,
# then set STOP_BLOB_H to that value.
STOP_BLOB_H = 150


def is_valid_green_target(blob):
    w = blob.w()
    h = blob.h()

    if w < MIN_BLOB_W or h < MIN_BLOB_H:
        return False
    if w > MAX_BLOB_W or h > MAX_BLOB_H:
        return False

    aspect_ratio = max(w / h, h / w)
    if aspect_ratio > MAX_ASPECT_RATIO:
        return False

    density = blob.pixels() / (w * h)
    if density < MIN_DENSITY:
        return False

    return True


csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.QVGA)
csi0.snapshot(time=2000)
csi0.auto_gain(False)
csi0.auto_whitebal(False)

clock = time.clock()
stable_count = 0
last_cx = None
last_cy = None

while True:
    clock.tick()
    img = csi0.snapshot()

    blobs = img.find_blobs(
        [GREEN_THRESHOLDS],
        pixels_threshold=PIXELS_THRESHOLD,
        area_threshold=AREA_THRESHOLD,
        merge=True,
    )
    valid_blobs = [blob for blob in blobs if is_valid_green_target(blob)]

    if valid_blobs:
        target = max(valid_blobs, key=lambda b: b.pixels())
        cx = target.cx()
        cy = target.cy()

        if last_cx is not None and abs(cx - last_cx) <= MAX_CENTER_JUMP and abs(cy - last_cy) <= MAX_CENTER_JUMP:
            stable_count = min(STABLE_FRAMES, stable_count + 1)
        else:
            stable_count = 1

        last_cx = cx
        last_cy = cy

        if stable_count >= STABLE_FRAMES:
            img.draw_rectangle(target.x(), target.y(), target.w(), target.h(), color=(0, 255, 0))
            img.draw_cross(cx, cy, color=(0, 255, 0))
            img.draw_string(0, 0, "GREEN", color=(0, 255, 0))
            x_error = X_SIGN * (cx - FRAME_CENTER_X - X_OFFSET)
            stop_flag = target.h() >= STOP_BLOB_H
            print("GREEN", cx, cy, target.w(), target.h(), target.pixels(), x_error, stop_flag)
        else:
            print("GREEN WAIT")
    else:
        stable_count = 0
        last_cx = None
        last_cy = None
        print("GREEN 0")

    print(clock.fps())
