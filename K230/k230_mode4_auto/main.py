"""Minimal K230 steel-ball detector with automatic Mode-4 startup.

The first three valid ball packets are sent to Tianmengxing, then MODE,4 is
sent once.  Afterwards the K230 only keeps transmitting BALL/NOBALL packets.
There is no touch menu, Wi-Fi, RTSP, or YOLO in this program.
"""

from machine import FPIOA, UART
from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor
import cv2
import gc
import time


AUTO_MODE = 4
MODE_START_VALID_PACKETS = 3

SENSOR_ID = 2
FRAME_WIDTH = 800
FRAME_HEIGHT = 480
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480

# Current fixed pipe ROI.
ROI_CENTER_Y_NORM = 0.405
ROI_HALF_HEIGHT = 38

# OpenCV Hough-circle parameters verified with the 1 cm steel ball.
BLUR_KERNEL = (7, 7)
BLUR_SIGMA = 1.5
HOUGH_DP = 1.2
HOUGH_MIN_DISTANCE = 30
HOUGH_CANNY_THRESHOLD = 90
HOUGH_ACCUMULATOR_THRESHOLD = 22
FALLBACK_HOUGH_CANNY_THRESHOLD = 70
FALLBACK_HOUGH_ACCUMULATOR_THRESHOLD = 18
FIXED_RADIUS = 17
MIN_RADIUS = 12
MAX_RADIUS = 23
TRACK_X_COST_DIVISOR = 8
ROI_CIRCLE_EDGE_MARGIN = 2

# Current left/zero/right position calibration.
CALIBRATION_X_NEG10 = 17.0
CALIBRATION_X_ZERO = 407.0
CALIBRATION_X_POS10 = 795.0
CALIBRATION_DISTANCE_CM = 10.0
DISPLAY_MARK_X_NEG5 = 198
DISPLAY_MARK_X_POS5 = 594

# Lushan Pi K230 GH1.25-4P UART3.
# K230 T(GPIO50) -> Tianmengxing RX, K230 R(GPIO51) <- Tianmengxing TX.
UART3_TX_PIN = 50
UART3_RX_PIN = 51
UART_BAUDRATE = 115200
UART_SEND_INTERVAL_MS = 20
NOBALL_CONFIRM_PACKETS = 3

# Keep this False until a suitable value has been measured under the final
# LED lighting.  Suggested starting range: 8000..12000 us.
USE_MANUAL_EXPOSURE = False
MANUAL_EXPOSURE_US = 10000

PRINT_EVERY_N_FRAMES = 10
GC_EVERY_N_FRAMES = 20
DISPLAY_HOLD_FRAMES = 2


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


def find_best_circle(image_array, roi_y0, roi_y1, last_center_x):
    """Return best circle, count, and whether fallback processing was used."""
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

    fallback_used = False
    if circles is None:
        # Only pay the cost of histogram equalization and the looser Hough
        # pass after the normal detector has failed.
        fallback_used = True
        equalized = cv2.equalizeHist(gray)
        fallback_blurred = cv2.GaussianBlur(
            equalized, BLUR_KERNEL, BLUR_SIGMA
        )
        circles = cv2.HoughCircles(
            fallback_blurred,
            3,
            dp=HOUGH_DP,
            minDist=HOUGH_MIN_DISTANCE,
            param1=FALLBACK_HOUGH_CANNY_THRESHOLD,
            param2=FALLBACK_HOUGH_ACCUMULATOR_THRESHOLD,
            minRadius=MIN_RADIUS,
            maxRadius=MAX_RADIUS,
        )

    if circles is None:
        return None, 0, fallback_used

    circle_shape = circles.shape
    if len(circle_shape) == 0 or circle_shape[-1] < 3:
        return None, 0, fallback_used

    circle_count = 1
    for dimension in circle_shape[:-1]:
        circle_count *= dimension
    circle_table = circles.reshape((circle_count, circle_shape[-1]))

    roi_center_y = (roi_y0 + roi_y1) // 2
    best = None
    best_cost = None
    for index in range(circle_count):
        center_x = int(round(float(circle_table[index, 0])))
        center_y = int(round(float(circle_table[index, 1]))) + roi_y0
        detected_radius = int(round(float(circle_table[index, 2])))
        # Reject partial circles attached to hardware just outside the ROI.
        if (
            center_y - detected_radius <
            roi_y0 + ROI_CIRCLE_EDGE_MARGIN
            or center_y + detected_radius >=
            roi_y1 - ROI_CIRCLE_EDGE_MARGIN
        ):
            continue
        cost = (
            abs(center_y - roi_center_y) * 2
            + abs(detected_radius - FIXED_RADIUS)
        )
        if last_center_x is not None:
            cost += abs(center_x - last_center_x) // TRACK_X_COST_DIVISOR
        if best is None or cost < best_cost:
            best = (
                center_x,
                center_y,
                FIXED_RADIUS,
                position_from_x(center_x),
            )
            best_cost = cost

    return best, circle_count, fallback_used


def send_control_packet(uart, best, mode_sent, valid_packets, lost_packets):
    """Send one position packet and start Mode 4 once detection is stable."""
    if best is None:
        if lost_packets < NOBALL_CONFIRM_PACKETS:
            lost_packets += 1
        if lost_packets >= NOBALL_CONFIRM_PACKETS:
            uart.write(b"NOBALL\n")
        return mode_sent, 0, lost_packets

    position_x100 = int(round(best[3] * 100.0))
    uart.write(("BALL,%d\n" % position_x100).encode())
    lost_packets = 0
    if valid_packets < MODE_START_VALID_PACKETS:
        valid_packets += 1

    if (not mode_sent) and valid_packets >= MODE_START_VALID_PACKETS:
        uart.write(("MODE,%d\n" % AUTO_MODE).encode())
        mode_sent = True
        print("[UART] MODE,%d sent" % AUTO_MODE)

    return mode_sent, valid_packets, lost_packets


def draw_results(image, roi_y0, roi_y1, best, display_best,
                 display_hold, circle_count, fps, mode_sent,
                 fallback_used, lost_packets):
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

    if display_best is None:
        position_text = "CV position: -- cm"
    else:
        center_x, center_y, radius, position_cm = display_best
        circle_color = (255, 128, 0) if display_hold else (0, 255, 0)
        image.draw_circle(
            center_x,
            center_y,
            radius,
            color=circle_color,
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

    mode_text = "MODE4: RUNNING" if mode_sent else "MODE4: WAIT BALL"
    mode_color = (0, 255, 0) if mode_sent else (255, 255, 0)
    if display_hold:
        detect_text = "CV: HOLD"
        detect_color = (255, 128, 0)
    elif best is None:
        detect_text = "CV: LOST %d/%d" % (
            lost_packets, NOBALL_CONFIRM_PACKETS
        )
        detect_color = (255, 0, 0)
    elif fallback_used:
        detect_text = "CV: FALLBACK"
        detect_color = (255, 128, 0)
    else:
        detect_text = "CV: NORMAL"
        detect_color = (0, 255, 0)
    image.draw_string_advanced(
        5, 5, 20, position_text, color=(255, 255, 0)
    )
    image.draw_string_advanced(
        5,
        30,
        18,
        "circles: %d  FPS: %.1f" % (circle_count, fps),
        color=(0, 255, 255),
    )
    image.draw_string_advanced(
        5, 52, 18, mode_text, color=mode_color
    )
    image.draw_string_advanced(
        5, 74, 18, detect_text, color=detect_color
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
        if USE_MANUAL_EXPOSURE:
            sensor.auto_exposure(False)
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
        if USE_MANUAL_EXPOSURE:
            sensor.exposure(MANUAL_EXPOSURE_US)
            print("[CV] fixed exposure: %d us" % MANUAL_EXPOSURE_US)

        print(
            "[CV] Mode4 auto ready; ROI y=%d..%d radius=%d..%d"
            % (roi_y0, roi_y1, MIN_RADIUS, MAX_RADIUS)
        )

        frame_index = 0
        next_uart_send_ms = time.ticks_ms()
        mode_sent = False
        valid_packets = 0
        lost_packets = 0
        last_center_x = None
        last_display_best = None
        display_miss_frames = 0
        clock = time.clock()

        while True:
            clock.tick()
            image = sensor.snapshot()
            image_array = image.to_numpy_ref()
            best, circle_count, fallback_used = find_best_circle(
                image_array, roi_y0, roi_y1, last_center_x
            )
            fps = clock.fps()

            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, next_uart_send_ms) >= 0:
                mode_sent, valid_packets, lost_packets = send_control_packet(
                    uart,
                    best,
                    mode_sent,
                    valid_packets,
                    lost_packets,
                )
                next_uart_send_ms = time.ticks_add(
                    now_ms, UART_SEND_INTERVAL_MS
                )

            if best is not None:
                last_center_x = best[0]
                last_display_best = best
                display_miss_frames = 0
                display_best = best
                display_hold = False
            elif lost_packets >= NOBALL_CONFIRM_PACKETS:
                last_center_x = None

            if best is None:
                display_miss_frames += 1
                if (
                    last_display_best is not None
                    and display_miss_frames <= DISPLAY_HOLD_FRAMES
                ):
                    display_best = last_display_best
                    display_hold = True
                else:
                    display_best = None
                    display_hold = False

            draw_results(
                image,
                roi_y0,
                roi_y1,
                best,
                display_best,
                display_hold,
                circle_count,
                fps,
                mode_sent,
                fallback_used,
                lost_packets,
            )

            if frame_index % PRINT_EVERY_N_FRAMES == 0:
                if best is None:
                    print(
                        "[CV] circles=%d position=NA mode4=%d fps=%.1f"
                        % (circle_count, mode_sent, fps)
                    )
                else:
                    print(
                        "[CV] center=(%d,%d) r=%d position=%+.2fcm "
                        "mode4=%d fps=%.1f"
                        % (
                            best[0],
                            best[1],
                            best[2],
                            best[3],
                            mode_sent,
                            fps,
                        )
                    )

            Display.show_image(image)
            frame_index += 1
            if frame_index % GC_EVERY_N_FRAMES == 0:
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
