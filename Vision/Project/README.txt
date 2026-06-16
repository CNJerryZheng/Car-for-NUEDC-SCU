OpenMV green target recognition project

Task:
- Detect the green target block/pillar placed beside the lane guide line.
- Output the target horizontal error to STM32 through UART.
- This project only contains the vision + UART output part.

Main file:
- green_uart_tx.py

Debug file:
- green_blob_tracking.py
- Same green detection logic, but only draws the box/cross and prints debug data.
- It does not send UART packets.

Color threshold:
- GREEN_THRESHOLDS = (20, 90, -80, -10, -10, 80)
- This is the only parameter that normally needs on-site tuning.
- Keep auto gain and auto white balance disabled when tuning.

Target filtering:
- The code rejects tiny noise, overly large background regions, long thin reflections, and sparse blobs.
- Adjustable parameters:
  PIXELS_THRESHOLD
  AREA_THRESHOLD
  MIN_BLOB_W / MIN_BLOB_H
  MAX_BLOB_W / MAX_BLOB_H
  MAX_ASPECT_RATIO
  MIN_DENSITY

UART:
- UART(3), 115200 baud
- OpenMV P4 = TX
- OpenMV P5 = RX

Protocol:
- Found green target:
  0x55, 0x01, x_error_high, x_error_low, stop_flag, 0xAA
- No target:
  0x55, 0x00, 0x00, 0x00, 0x00, 0xAA

x_error:
- x_error = target_center_x - 160
- Positive means the target is on the right.
- Negative means the target is on the left.
- int16, high byte first.

Timing:
- One packet every 20 ms.

stop_flag:
- 0 means the target has not reached the stop distance.
- 1 means the target is close enough to stop.
- OpenMV uses target image height to estimate distance.
- Put the car 40 mm from the green target, read the printed blob height, then set STOP_BLOB_H to that value.

STM32 receive note:
- The packet length is now 6 bytes.
- rx_frame should be uint8_t rx_frame[6].
- A complete packet is valid when rx_frame[0] == 0x55 and rx_frame[5] == 0xAA.
- stop_flag is rx_frame[4].
