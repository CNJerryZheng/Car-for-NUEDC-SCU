# Green target detection with UART output for STM32.
# Protocol: 0x55, flag, high_byte, low_byte, stop_flag, 0xAA

import csi
import time
from pyb import UART

# LAB threshold for green targets.
# Only this value normally needs on-site tuning in OpenMV IDE.
GREEN_THRESHOLDS = (20, 90, -80, -10, -10, 80)

# Reject small noise and large background regions.
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

UART_PORT = 1
UART_BAUD = 115200
FRAME_HEADER = 0x55
FRAME_TAIL = 0xAA

FRAME_WIDTH = 320
FRAME_CENTER_X = FRAME_WIDTH // 2
SEND_INTERVAL_MS = 20

# Camera mounting calibration.
# If the car turns the wrong way, set X_SIGN to -1.
# If the target is physically centered but x_error is not 0, set X_OFFSET to that value.
X_SIGN = 1
X_OFFSET = 0

# Stop-distance calibration.
# OpenMV cannot directly measure 40 mm with one camera.
# Put the car 40 mm from the green target, read the printed blob height,
# then set STOP_BLOB_H to that value.
STOP_BLOB_H = 100

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


def send_packet(uart, found, x_error, stop_flag):
    stop_flag = 0x01 if stop_flag else 0x00
    if found:
        x_error = int(max(-32768, min(32767, x_error)))
        high_byte = (x_error >> 8) & 0xFF
        low_byte = x_error & 0xFF
        uart.write(bytearray([FRAME_HEADER, 0x01, high_byte, low_byte, stop_flag, FRAME_TAIL]))
    else:
        uart.write(bytearray([FRAME_HEADER, 0x00, 0x00, 0x00, 0x00, FRAME_TAIL]))


csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.QVGA)
csi0.snapshot(time=2000)
csi0.auto_gain(False)
csi0.auto_whitebal(False)

uart = UART(UART_PORT, UART_BAUD, timeout_char=200)
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

            x_error = X_SIGN * (cx - FRAME_CENTER_X - X_OFFSET)
            stop_flag = target.h() >= STOP_BLOB_H
            send_packet(uart, True, x_error, stop_flag)
            print("GREEN", cx, cy, target.w(), target.h(), target.pixels(), x_error, stop_flag)
        else:
            send_packet(uart, False, 0, False)
            print("GREEN WAIT")
    else:
        stable_count = 0
        last_cx = None
        last_cy = None
        send_packet(uart, False, 0, False)
        print("GREEN 0")

    time.sleep_ms(SEND_INTERVAL_MS)
