"""K230 OpenCV circle-detection test for the steel ball.

This is an independent comparison program.  It does not use the YOLO model and
does not replace main.py.  The Lushan Pi K230 UART3 packet format is kept the
same so Tianmengxing STATUS can be used during the comparison.
"""

from machine import FPIOA, UART
from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor
from ulab import numpy as np
import cv2
import gc
import time


SENSOR_ID = 2
FRAME_WIDTH = 800
FRAME_HEIGHT = 480
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480

# Only run Hough circle detection in this horizontal strip.  Increase
# ROI_HALF_HEIGHT if the pipe is not fully covered.
ROI_CENTER_Y_NORM = 0.42
ROI_HALF_HEIGHT = 48

# Hough-circle tuning parameters.  A 1 cm ball occupying a 24 cm horizontal
# range is expected to have a radius of roughly 13-21 pixels in this image.
BLUR_KERNEL = (7, 7)
BLUR_SIGMA = 1.5
HOUGH_DP = 1.2
HOUGH_MIN_DISTANCE = 30
HOUGH_CANNY_THRESHOLD = 90
HOUGH_ACCUMULATOR_THRESHOLD = 22
FIXED_RADIUS = 17
MIN_RADIUS = 12
MAX_RADIUS = 23

CALIBRATION_X_NEG10 = 17.0
CALIBRATION_X_ZERO = 407.0
CALIBRATION_X_POS10 = 795.0
CALIBRATION_DISTANCE_CM = 10.0
DISPLAY_MARK_X_NEG5 = 198
DISPLAY_MARK_X_POS5 = 594

UART3_TX_PIN = 50
UART3_RX_PIN = 51
UART_BAUDRATE = 115200
UART_SEND_INTERVAL_MS = 50
PRINT_EVERY_N_FRAMES = 10


def create_control_uart():
    fpioa = FPIOA()
    fpioa.set_function(UART3_TX_PIN, FPIOA.UART3_TXD)
    fpioa.set_function(UART3_RX_PIN, FPIOA.UART3_RXD)
    return UART(
        UART.UART3,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


def position_from_x(center_x):
    if center_x <= CALIBRATION_X_ZERO:
        return (
            (center_x - CALIBRATION_X_ZERO)
            * CALIBRATION_DISTANCE_CM
            / (CALIBRATION_X_ZERO - CALIBRATION_X_NEG10)
        )
    return (
        (center_x - CALIBRATION_X_ZERO)
        * CALIBRATION_DISTANCE_CM
        / (CALIBRATION_X_POS10 - CALIBRATION_X_ZERO)
    )


def find_best_circle(image_array, roi_y0, roi_y1):
    """Return the best (x, y, radius, position_cm), plus all candidates."""
    # Slice only the first (row) axis.  This works with both the 2-D and 3-D
    # ndarray views returned by different CanMV firmware builds.
    roi = image_array[roi_y0:roi_y1]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, BLUR_KERNEL, BLUR_SIGMA)
    circles = cv2.HoughCircles(
        blurred,
        3,
        dp=HOUGH_DP,
        minDist=HOUGH_MIN_DISTANCE,
        param1=HOUGH_CANNY_THRESHOLD,
        param2=HOUGH_ACCUMULATOR_THRESHOLD,
        minRadius=MIN_RADIUS,
        maxRadius=MAX_RADIUS,
    )

    candidates = []
    best = None
    best_cost = None
    if circles is None:
        return best, candidates

    # Depending on the firmware, HoughCircles may return (N, 3), (N, 1, 3),
    # or (1, N, 3).  Collapse every leading dimension and keep x/y/r columns.
    circle_shape = circles.shape
    if len(circle_shape) == 0 or circle_shape[-1] < 3:
        return best, candidates
    circle_count = 1
    for dimension in circle_shape[:-1]:
        circle_count *= dimension
    circle_table = circles.reshape((circle_count, circle_shape[-1]))

    roi_center_y = (roi_y0 + roi_y1) // 2
    for index in range(circle_count):
        center_x = int(round(float(circle_table[index, 0])))
        center_y = int(round(float(circle_table[index, 1]))) + roi_y0
        detected_radius = int(round(float(circle_table[index, 2])))
        # Drawing and position output use one fixed steel-ball radius.
        radius = FIXED_RADIUS
        position_cm = position_from_x(center_x)
        candidate = (center_x, center_y, radius, position_cm)
        candidates.append(candidate)

        # Prefer a center near the ball track and a detected radius near 17 px.
        cost = (
            abs(center_y - roi_center_y) * 2
            + abs(detected_radius - FIXED_RADIUS)
        )
        if best is None or cost < best_cost:
            best = candidate
            best_cost = cost

    return best, candidates


def send_position_packet(uart, best):
    if best is None:
        uart.write(b"NOBALL\n")
    else:
        position_x100 = int(round(best[3] * 100.0))
        uart.write(("BALL,%d\n" % position_x100).encode())


def draw_results(image, roi_y0, roi_y1, best, candidates, fps):
    roi_center_y = (roi_y0 + roi_y1) // 2
    image.draw_rectangle(
        0,
        roi_y0,
        FRAME_WIDTH - 1,
        roi_y1 - roi_y0,
        color=(0, 255, 0),
        thickness=2,
    )
    image.draw_line(
        0,
        roi_center_y,
        FRAME_WIDTH - 1,
        roi_center_y,
        color=(0, 160, 255),
        thickness=2,
    )
    image.draw_line(
        FRAME_WIDTH // 2,
        roi_y0,
        FRAME_WIDTH // 2,
        roi_y1,
        color=(255, 0, 255),
        thickness=3,
    )
    for mark_x, mark_text in (
        (DISPLAY_MARK_X_NEG5, "-5"),
        (DISPLAY_MARK_X_POS5, "+5"),
    ):
        image.draw_line(
            mark_x,
            roi_y0,
            mark_x,
            roi_y1,
            color=(255, 255, 0),
            thickness=3,
        )
        image.draw_string_advanced(
            mark_x - 12,
            roi_y0 + 4,
            16,
            mark_text,
            color=(255, 255, 0),
        )

    for candidate in candidates:
        center_x, center_y, radius, _ = candidate
        image.draw_circle(
            center_x,
            center_y,
            radius,
            color=(255, 128, 0),
            thickness=1,
        )

    if best is None:
        position_text = "CV position: -- cm"
    else:
        center_x, center_y, radius, position_cm = best
        image.draw_circle(
            center_x,
            center_y,
            radius,
            color=(0, 255, 0),
            thickness=4,
        )
        image.draw_cross(
            center_x,
            center_y,
            color=(255, 255, 0),
            size=12,
            thickness=3,
        )
        position_text = "CV position: %+.2f cm" % position_cm

    image.draw_string_advanced(
        5, 5, 20, position_text, color=(255, 255, 0)
    )
    image.draw_string_advanced(
        5,
        30,
        18,
        "circles: %d  FPS: %.1f" % (len(candidates), fps),
        color=(0, 255, 255),
    )


def main():
    sensor = None
    uart = None
    media_initialized = False
    display_initialized = False

    roi_center_y = int(round(ROI_CENTER_Y_NORM * (FRAME_HEIGHT - 1)))
    roi_y0 = max(0, roi_center_y - ROI_HALF_HEIGHT)
    roi_y1 = min(FRAME_HEIGHT, roi_center_y + ROI_HALF_HEIGHT)

    try:
        uart = create_control_uart()

        sensor = Sensor(id=SENSOR_ID)
        sensor.reset()
        sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.set_pixformat(Sensor.RGB888)

        Display.init(
            Display.ST7701,
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            to_ide=True,
        )
        display_initialized = True
        MediaManager.init()
        media_initialized = True
        sensor.run()

        print(
            "[CV] UART3 ready; ROI y=%d..%d radius=%d..%d"
            % (roi_y0, roi_y1, MIN_RADIUS, MAX_RADIUS)
        )

        frame_index = 0
        next_uart_send_ms = time.ticks_ms()
        clock = time.clock()
        while True:
            clock.tick()
            image = sensor.snapshot()
            image_array = image.to_numpy_ref()
            best, candidates = find_best_circle(
                image_array, roi_y0, roi_y1
            )
            fps = clock.fps()
            draw_results(
                image, roi_y0, roi_y1, best, candidates, fps
            )

            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, next_uart_send_ms) >= 0:
                send_position_packet(uart, best)
                next_uart_send_ms = time.ticks_add(
                    now_ms, UART_SEND_INTERVAL_MS
                )

            if frame_index % PRINT_EVERY_N_FRAMES == 0:
                if best is None:
                    print(
                        "[CV] circles=%d position=NA fps=%.1f"
                        % (len(candidates), fps)
                    )
                else:
                    print(
                        "[CV] circles=%d center=(%d,%d) r=%d "
                        "position=%+.2fcm fps=%.1f"
                        % (
                            len(candidates),
                            best[0],
                            best[1],
                            best[2],
                            best[3],
                            fps,
                        )
                    )

            Display.show_image(image)
            frame_index += 1
            gc.collect()

    except KeyboardInterrupt:
        print("[CV] stopped by user")
    except BaseException as error:
        print("[CV] error:", error)
        raise
    finally:
        if sensor is not None:
            try:
                sensor.stop()
            except BaseException as cleanup_error:
                print("[CV] sensor cleanup error:", cleanup_error)
        if uart is not None:
            try:
                uart.deinit()
            except BaseException as cleanup_error:
                print("[CV] UART cleanup error:", cleanup_error)
        if display_initialized:
            try:
                Display.deinit()
            except BaseException as cleanup_error:
                print("[CV] display cleanup error:", cleanup_error)
        if media_initialized:
            try:
                MediaManager.deinit()
            except BaseException as cleanup_error:
                print("[CV] media cleanup error:", cleanup_error)
        gc.collect()


main()
