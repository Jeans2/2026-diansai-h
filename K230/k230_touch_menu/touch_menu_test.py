"""Lushan Pi K230 touch menu for raw RTSP and ball-control modes."""

import gc
import network
import os
import sys
import time
import uctypes
import multimedia as mm

from machine import FPIOA, TOUCH, UART
from media.display import *
from media.media import *
from media.sensor import *
from media.vencoder import *
from ulab import numpy as np
import cv2


DISPLAY_WIDTH = ALIGN_UP(800, 16)
DISPLAY_HEIGHT = 480

# Mode 1: the previously verified standalone H.264/RTSP configuration.
SENSOR_ID = 2
AP_SSID = "K230-Video"
AP_PASSWORD = "12345678"
AP_CHANNEL = 11
RTSP_PORT = 8554
RTSP_SESSION = "live"
SENSOR_WIDTH = 1920
SENSOR_HEIGHT = 1080
SENSOR_FPS = 30
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 360
VIDEO_BITRATE_KBPS = 1500
VIDEO_GOP = 15
VENC_BUFFER_COUNT = 4
STATS_PERIOD_MS = 2000
GC_PERIOD_MS = 10000

# Modes 3/4/5 use camera channel 2 for local CV and UART control. Camera
# channel 0 remains raw YUV and is the only channel sent to RTSP.
CV_FRAME_WIDTH = 800
CV_FRAME_HEIGHT = 480
ROI_CENTER_Y_NORM = 0.405
ROI_HALF_HEIGHT = 38
BLUR_KERNEL = (7, 7)
BLUR_SIGMA = 1.5
HOUGH_DP = 1.2
HOUGH_MIN_DISTANCE = 30
HOUGH_CANNY_THRESHOLD = 70
HOUGH_ACCUMULATOR_THRESHOLD = 18
FIXED_RADIUS = 17
MIN_RADIUS = 10
MAX_RADIUS = 26
TRACK_X_COST_DIVISOR = 8
ROI_CIRCLE_EDGE_MARGIN = 2
CALIBRATION_X_NEG10 = 17.0
CALIBRATION_X_ZERO = 407.0
CALIBRATION_X_POS10 = 795.0
CALIBRATION_DISTANCE_CM = 10.0
DISPLAY_MARK_X_NEG5 = 198
DISPLAY_MARK_X_POS5 = 594
MODE6_TARGET_X_MIN = int(CALIBRATION_X_NEG10)
MODE6_TARGET_X_MAX = int(CALIBRATION_X_POS10)
UART3_TX_PIN = 50
UART3_RX_PIN = 51
UART_BAUDRATE = 115200
UART_SEND_INTERVAL_MS = 20
MODE_START_VALID_PACKETS = 3
NOBALL_CONFIRM_PACKETS = 3
DISPLAY_HOLD_FRAMES = 2
PRINT_EVERY_N_FRAMES = 10

MODE_IDS = (1, 2, 3, 4, 5, 6)
MODE_TITLES = (
    "MODE 1  Video",
    "MODE 2  Tracking",
    "MODE 3  +/-5cm",
    "MODE 4  Line 0cm",
    "MODE 5  Lap 0cm",
    "MODE 6  Target",
)
MODE_DETAILS = (
    "H.264 / RTSP video transmission",
    "Raw video; tracking MCU works alone",
    "Ball: 0 -> +5cm -> -5cm",
    "Straight track, keep ball at 0cm",
    "Full lap, keep ball at 0cm",
    "Full lap, keep ball at selected target",
)

MODE_BUTTON_X = 24
MODE_BUTTON_Y = 60
MODE_BUTTON_W = 310
MODE_BUTTON_H = 54
MODE_BUTTON_GAP = 6

PANEL_X = 360
PANEL_Y = 68
PANEL_W = 416
PANEL_H = 260

START_RECT = (382, 350, 180, 88)
STOP_RECT = (584, 350, 170, 88)
TASK_ARM_RECT = (580, 390, 200, 70)

COLOR_BACKGROUND = (18, 24, 36)
COLOR_HEADER = (33, 46, 68)
COLOR_PANEL = (27, 37, 54)
COLOR_BUTTON = (48, 63, 86)
COLOR_SELECTED = (24, 142, 110)
COLOR_RUNNING = (30, 174, 110)
COLOR_STOP = (196, 70, 70)
COLOR_TEXT = (245, 248, 252)
COLOR_MUTED = (165, 178, 198)
COLOR_BORDER = (93, 116, 148)
COLOR_TOUCH = (255, 214, 74)


def print_exception_compatible(error):
    printer = getattr(sys, "print_exception", None)
    if printer:
        printer(error)
    else:
        print("[MENU] exception:", repr(error))


def mode_rect(index):
    return (
        MODE_BUTTON_X,
        MODE_BUTTON_Y + index * (MODE_BUTTON_H + MODE_BUTTON_GAP),
        MODE_BUTTON_W,
        MODE_BUTTON_H,
    )


def point_in_rect(x, y, rect):
    left, top, width, height = rect
    return (
        x >= left and x < (left + width)
        and y >= top and y < (top + height)
    )


def draw_filled_box(canvas, rect, fill_color, border_color=COLOR_BORDER):
    x, y, width, height = rect
    canvas.draw_rectangle(
        x, y, width, height, color=fill_color, thickness=1, fill=True
    )
    canvas.draw_rectangle(
        x, y, width, height, color=border_color, thickness=2, fill=False
    )


def draw_menu(canvas, selected_index, running_mode, status_text,
              last_touch):
    canvas.clear()
    canvas.draw_rectangle(
        0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT,
        color=COLOR_BACKGROUND, thickness=1, fill=True
    )
    canvas.draw_rectangle(
        0, 0, DISPLAY_WIDTH, 52,
        color=COLOR_HEADER, thickness=1, fill=True
    )
    canvas.draw_string_advanced(
        24, 10, 30, "K230 Touch Task Menu", color=COLOR_TEXT
    )
    canvas.draw_string_advanced(
        570, 15, 20, "Select then START", color=COLOR_MUTED
    )

    for index in range(len(MODE_IDS)):
        selected = index == selected_index
        fill_color = COLOR_SELECTED if selected else COLOR_BUTTON
        draw_filled_box(canvas, mode_rect(index), fill_color)
        canvas.draw_string_advanced(
            MODE_BUTTON_X + 16,
            mode_rect(index)[1] + 17,
            24,
            MODE_TITLES[index],
            color=COLOR_TEXT,
        )

    draw_filled_box(canvas, (PANEL_X, PANEL_Y, PANEL_W, PANEL_H), COLOR_PANEL)
    selected_mode = MODE_IDS[selected_index]
    canvas.draw_string_advanced(
        PANEL_X + 22, PANEL_Y + 22, 34,
        "Selected: MODE %d" % selected_mode,
        color=COLOR_TOUCH,
    )
    canvas.draw_string_advanced(
        PANEL_X + 22, PANEL_Y + 82, 24,
        MODE_DETAILS[selected_index],
        color=COLOR_TEXT,
    )

    if running_mode is None:
        running_text = "Running: none"
        running_color = COLOR_MUTED
    else:
        running_text = "Running: MODE %d" % running_mode
        running_color = COLOR_RUNNING
    canvas.draw_string_advanced(
        PANEL_X + 22, PANEL_Y + 140, 26,
        running_text, color=running_color,
    )
    canvas.draw_string_advanced(
        PANEL_X + 22, PANEL_Y + 190, 20,
        status_text, color=COLOR_MUTED,
    )

    draw_filled_box(canvas, START_RECT, COLOR_RUNNING)
    canvas.draw_string_advanced(
        START_RECT[0] + 18, START_RECT[1] + 28,
        27, "OPEN RTSP", color=COLOR_TEXT,
    )

    draw_filled_box(canvas, STOP_RECT, COLOR_STOP)
    canvas.draw_string_advanced(
        STOP_RECT[0] + 37, STOP_RECT[1] + 25,
        34, "STOP", color=COLOR_TEXT,
    )

    if last_touch is not None:
        touch_x, touch_y = last_touch
        canvas.draw_cross(
            touch_x, touch_y,
            color=COLOR_TOUCH, size=18, thickness=2,
        )
        canvas.draw_string_advanced(
            PANEL_X + 22, 445, 18,
            "Touch: x=%d y=%d" % (touch_x, touch_y),
            color=COLOR_MUTED,
        )


def handle_touch(x, y, selected_index, running_mode):
    for index in range(len(MODE_IDS)):
        if point_in_rect(x, y, mode_rect(index)):
            selected_index = index
            print("[MENU] selected MODE", MODE_IDS[selected_index])
            return selected_index, running_mode, "Mode selected"

    if point_in_rect(x, y, START_RECT):
        running_mode = MODE_IDS[selected_index]
        print("[MENU] START MODE", running_mode)
        return selected_index, running_mode, "START event sent"

    if point_in_rect(x, y, STOP_RECT):
        if running_mode is None:
            print("[MENU] STOP (already stopped)")
        else:
            print("[MENU] STOP MODE", running_mode)
        running_mode = None
        return selected_index, running_mode, "STOP event sent"

    print("[MENU] touch outside buttons: x=%d y=%d" % (x, y))
    return selected_index, running_mode, "Touch outside button"


def start_hotspot():
    wlan = network.WLAN(network.AP_IF)
    if not wlan.active():
        wlan.active(True)

    if wlan.config(ssid=AP_SSID, key=AP_PASSWORD) is False:
        raise RuntimeError("failed to create temporary Wi-Fi AP")
    time.sleep_ms(500)

    ap_info = wlan.info()
    ap_info.ssid = AP_SSID
    ap_info.channel = AP_CHANNEL
    ap_info.security = wlan.SECURITY_WPA2_AES_PSK
    ap_info.band = wlan.BAND_2_4GHZ
    ap_info.hidden = 0

    wlan.stop()
    time.sleep_ms(300)
    if wlan.config(info=ap_info, key=AP_PASSWORD) is False:
        raise RuntimeError("failed to create WPA2 Wi-Fi AP")
    time.sleep(3)

    ip = wlan.ifconfig()[0]
    if ip == "0.0.0.0":
        raise RuntimeError("Wi-Fi AP did not obtain an IP address")

    print("")
    print("========================================")
    print("[WiFi] hotspot ready")
    print("[WiFi] SSID:", AP_SSID)
    print("[WiFi] password:", AP_PASSWORD)
    print("[WiFi] network:", wlan.ifconfig())
    print("========================================")
    return wlan, ip


def create_encoder_attr(encoder, width, height):
    try:
        attr = ChnAttrStr(
            encoder.PAYLOAD_TYPE_H264,
            encoder.H264_PROFILE_BASELINE,
            width,
            height,
            VIDEO_BITRATE_KBPS,
            VIDEO_GOP,
            SENSOR_FPS,
            SENSOR_FPS,
        )
        print("[Video] encoder: %d kbps, GOP %d" % (
            VIDEO_BITRATE_KBPS, VIDEO_GOP))
        return attr
    except TypeError:
        print("[Video] old ChnAttrStr API; using firmware defaults")
        return ChnAttrStr(
            encoder.PAYLOAD_TYPE_H264,
            encoder.H264_PROFILE_BASELINE,
            width,
            height,
        )


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
    """Return best circle, accepted candidates, and fallback state."""
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

    candidates = []
    best = None
    best_cost = None
    if circles is None:
        return best, candidates, fallback_used

    circle_shape = circles.shape
    if len(circle_shape) == 0 or circle_shape[-1] < 3:
        return best, candidates, fallback_used
    circle_count = 1
    for dimension in circle_shape[:-1]:
        circle_count *= dimension
    circle_table = circles.reshape((circle_count, circle_shape[-1]))

    roi_center_y = (roi_y0 + roi_y1) // 2
    for index in range(circle_count):
        center_x = int(round(float(circle_table[index, 0])))
        center_y = int(round(float(circle_table[index, 1]))) + roi_y0
        detected_radius = int(round(float(circle_table[index, 2])))
        if (
            center_y - detected_radius <
            roi_y0 + ROI_CIRCLE_EDGE_MARGIN
            or center_y + detected_radius >=
            roi_y1 - ROI_CIRCLE_EDGE_MARGIN
        ):
            continue
        radius = FIXED_RADIUS
        position_cm = position_from_x(center_x)
        candidate = (center_x, center_y, radius, position_cm)
        candidates.append(candidate)
        cost = (
            abs(center_y - roi_center_y) * 2
            + abs(detected_radius - FIXED_RADIUS)
        )
        if last_center_x is not None:
            cost += abs(center_x - last_center_x) // TRACK_X_COST_DIVISOR
        if best is None or cost < best_cost:
            best = candidate
            best_cost = cost

    return best, candidates, fallback_used


def send_control_packet(uart, best, mode_id, mode_sent,
                        valid_packets, lost_packets,
                        mode6_target_x100, task_armed):
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

    if (task_armed and (not mode_sent)
            and valid_packets >= MODE_START_VALID_PACKETS):
        if mode_id == 6:
            uart.write(("TARGET,%d\n" % mode6_target_x100).encode())
        time.sleep_ms(2)
        uart.write(("MODE,%d\n" % mode_id).encode())
        mode_sent = True
        print("[UART] MODE,%d sent after stable ball" % mode_id)

    return mode_sent, valid_packets, lost_packets


def draw_task_gate(frame, mode_id, task_armed):
    if task_armed:
        frame.draw_string_advanced(
            610, 445, 18,
            "MODE %d RUNNING" % mode_id,
            color=COLOR_RUNNING,
        )
        return

    frame.draw_string_advanced(
        492, 358, 18,
        "Connect RTSP and start recording first",
        color=(0, 255, 255),
    )
    draw_filled_box(frame, TASK_ARM_RECT, COLOR_RUNNING)
    frame.draw_string_advanced(
        TASK_ARM_RECT[0] + 20,
        TASK_ARM_RECT[1] + 22,
        28,
        "START TASK",
        color=COLOR_TEXT,
    )


def draw_cv_results(frame, roi_y0, roi_y1, best, display_best,
                    display_hold, candidates, fps, fallback_used,
                    lost_packets, mode_id, mode6_target_x,
                    mode6_target_x100):
    """Draw only on the local CV frame; RTSP uses another camera channel."""
    roi_center_y = (roi_y0 + roi_y1) // 2
    frame.draw_rectangle(
        0, roi_y0, CV_FRAME_WIDTH - 1, roi_y1 - roi_y0,
        color=(0, 255, 0), thickness=2,
    )
    if mode_id == 6:
        frame.draw_line(
            mode6_target_x, roi_y0, mode6_target_x, roi_y1,
            color=(255, 0, 255), thickness=4,
        )
        label_x = max(4, min(mode6_target_x - 50, CV_FRAME_WIDTH - 150))
        frame.draw_string_advanced(
            label_x, roi_y0 + 4, 16,
            "target %+.2f" % (mode6_target_x100 / 100.0),
            color=(255, 0, 255),
        )
    else:
        frame.draw_line(
            0, roi_center_y, CV_FRAME_WIDTH - 1, roi_center_y,
            color=(0, 160, 255), thickness=2,
        )
        frame.draw_line(
            CV_FRAME_WIDTH // 2, roi_y0,
            CV_FRAME_WIDTH // 2, roi_y1,
            color=(255, 0, 255), thickness=3,
        )
        for mark_x, mark_text in (
            (DISPLAY_MARK_X_NEG5, "-5"),
            (DISPLAY_MARK_X_POS5, "+5"),
        ):
            frame.draw_line(
                mark_x, roi_y0, mark_x, roi_y1,
                color=(255, 255, 0), thickness=3,
            )
            frame.draw_string_advanced(
                mark_x - 12, roi_y0 + 4, 16, mark_text,
                color=(255, 255, 0),
            )

    if display_best is None:
        position_text = "CV position: -- cm"
    else:
        center_x, center_y, radius, position_cm = display_best
        circle_color = (255, 128, 0) if display_hold else (0, 255, 0)
        frame.draw_circle(
            center_x, center_y, radius,
            color=circle_color, thickness=4,
        )
        frame.draw_cross(
            center_x, center_y,
            color=(255, 255, 0), size=12, thickness=3,
        )
        position_text = "CV position: %+.2f cm" % position_cm

    frame.draw_string_advanced(
        5, 5, 20, position_text, color=(255, 255, 0)
    )
    frame.draw_string_advanced(
        5, 30, 18,
        "circles: %d  FPS: %.1f" % (len(candidates), fps),
        color=(0, 255, 255),
    )
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
    frame.draw_string_advanced(
        5, 52, 18, detect_text, color=detect_color
    )
    if mode_id == 6:
        frame.draw_string_advanced(
            5, 74, 18,
            "Touch pipe to move target: %+.2f cm"
            % (mode6_target_x100 / 100.0),
            color=(255, 0, 255),
        )


def run_raw_rtsp(mode_id):
    """Run raw video transmission until IDE stop or board reset."""
    wlan = None
    sensor = None
    encoder = None
    media_link = None
    rtsp = None

    media_ready = False
    encoder_created = False
    encoder_started = False
    rtsp_ready = False
    rtsp_started = False

    width = ALIGN_UP(VIDEO_WIDTH, 16)
    height = VIDEO_HEIGHT

    os.exitpoint(os.EXITPOINT_ENABLE)
    print("[MENU] entering MODE %d raw RTSP" % mode_id)

    try:
        wlan, ip = start_hotspot()

        sensor = Sensor(
            id=SENSOR_ID,
            width=SENSOR_WIDTH,
            height=SENSOR_HEIGHT,
            fps=SENSOR_FPS,
        )
        sensor.reset()
        sensor.set_framesize(
            width=width,
            height=height,
            chn=CAM_CHN_ID_0,
            alignment=12,
        )
        sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)

        encoder = Encoder()
        encoder.SetOutBufs(VENC_BUFFER_COUNT, width, height)
        encoder.Create(create_encoder_attr(encoder, width, height))
        encoder_created = True

        sensor_source = sensor.bind_info(chn=CAM_CHN_ID_0)["src"]
        encoder_destination = (
            VIDEO_ENCODE_MOD_ID,
            VENC_DEV_ID,
            encoder.chn,
        )
        media_link = MediaManager.link(sensor_source, encoder_destination)
        MediaManager.init()
        media_ready = True

        rtsp = mm.rtsp_server()
        rtsp.rtspserver_init(RTSP_PORT)
        rtsp_ready = True
        rtsp.rtspserver_createsession(
            RTSP_SESSION,
            mm.multi_media_type.media_h264,
            False,
        )
        rtsp.rtspserver_start()
        rtsp_started = True

        encoder.Start()
        encoder_started = True
        sensor.run()

        url = "rtsp://%s:%d/%s" % (ip, RTSP_PORT, RTSP_SESSION)
        print("")
        print("================================================")
        print("[RTSP] camera stream ready")
        print("[RTSP] VLC address:", url)
        print("[RTSP] video: %dx%d @ %d FPS, H.264" % (
            width, height, SENSOR_FPS))
        print("[RTSP] reset K230 to return to the touch menu")
        print("================================================")

        stream_data = StreamData()
        period_frames = 0
        period_bytes = 0
        total_frames = 0
        stats_started = time.ticks_ms()
        gc_started = stats_started

        while True:
            os.exitpoint()
            result = encoder.GetStream(stream_data)
            if result not in (0, None):
                time.sleep_ms(2)
                continue

            frame_bytes = 0
            frame_timestamp = time.ticks_ms()
            try:
                for index in range(stream_data.pack_cnt):
                    packet_size = stream_data.data_size[index]
                    if packet_size <= 0:
                        continue

                    packet = bytes(uctypes.bytearray_at(
                        stream_data.data[index], packet_size))
                    rtsp.rtspserver_sendvideodata(
                        RTSP_SESSION,
                        packet,
                        packet_size,
                        frame_timestamp,
                    )
                    frame_bytes += packet_size
            finally:
                encoder.ReleaseStream(stream_data)

            total_frames += 1
            period_frames += 1
            period_bytes += frame_bytes

            now = time.ticks_ms()
            elapsed = time.ticks_diff(now, stats_started)
            if elapsed >= STATS_PERIOD_MS:
                seconds = elapsed / 1000.0
                fps = period_frames / seconds
                mbps = period_bytes * 8.0 / seconds / 1000000.0
                print("[Stats] FPS: %.1f  bitrate: %.2f Mbps  frames: %d" % (
                    fps, mbps, total_frames))
                period_frames = 0
                period_bytes = 0
                stats_started = now

            if time.ticks_diff(now, gc_started) >= GC_PERIOD_MS:
                gc.collect()
                gc_started = now

    except KeyboardInterrupt:
        print("[MODE %d] stopped by IDE" % mode_id)
    except BaseException as error:
        print("[MODE %d] error:" % mode_id, repr(error))
        raise
    finally:
        print("[MODE %d] releasing resources" % mode_id)

        if sensor is not None:
            try:
                sensor.stop()
            except BaseException as error:
                print("[RTSP] sensor:", repr(error))

        if media_link is not None:
            try:
                media_link.destroy()
            except BaseException:
                pass

        if encoder_started and encoder is not None:
            try:
                encoder.Stop()
            except BaseException as error:
                print("[RTSP] encoder stop:", repr(error))

        if encoder_created and encoder is not None:
            try:
                encoder.Destroy()
            except BaseException as error:
                print("[RTSP] encoder destroy:", repr(error))

        if rtsp_started and rtsp is not None:
            try:
                rtsp.rtspserver_stop()
            except BaseException as error:
                print("[RTSP] stop:", repr(error))

        if rtsp_ready and rtsp is not None:
            try:
                rtsp.rtspserver_destroysession(RTSP_SESSION)
            except BaseException:
                pass
            try:
                rtsp.rtspserver_deinit()
            except BaseException as error:
                print("[RTSP] deinit:", repr(error))

        if media_ready:
            try:
                time.sleep_ms(100)
                MediaManager.deinit()
            except BaseException as error:
                print("[RTSP] media:", repr(error))

        if wlan is not None:
            try:
                wlan.stop()
            except BaseException as error:
                print("[RTSP] Wi-Fi:", repr(error))

        gc.collect()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        print("[MODE %d] finished" % mode_id)


def run_ball_mode_with_raw_rtsp(mode_id):
    """Run local CV/UART control while RTSP sends untouched camera frames."""
    wlan = None
    sensor = None
    encoder = None
    media_link = None
    rtsp = None
    uart = None
    touch = None

    media_ready = False
    display_ready = False
    encoder_created = False
    encoder_started = False
    rtsp_ready = False
    rtsp_started = False

    video_width = ALIGN_UP(VIDEO_WIDTH, 16)
    video_height = VIDEO_HEIGHT
    roi_center_y = int(round(ROI_CENTER_Y_NORM * (CV_FRAME_HEIGHT - 1)))
    roi_y0 = max(0, roi_center_y - ROI_HALF_HEIGHT)
    roi_y1 = min(CV_FRAME_HEIGHT, roi_center_y + ROI_HALF_HEIGHT)

    os.exitpoint(os.EXITPOINT_ENABLE)
    print("[MENU] entering MODE %d CV + raw RTSP" % mode_id)

    try:
        uart = create_control_uart()
        wlan, ip = start_hotspot()

        sensor = Sensor(
            id=SENSOR_ID,
            width=SENSOR_WIDTH,
            height=SENSOR_HEIGHT,
            fps=SENSOR_FPS,
        )
        sensor.reset()

        # Channel 0 is bound straight to H.264. No snapshot and no drawing.
        sensor.set_framesize(
            width=video_width,
            height=video_height,
            chn=CAM_CHN_ID_0,
            alignment=12,
        )
        sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)

        # Channel 2 is independent and is used only by CV/local display.
        sensor.set_framesize(
            width=CV_FRAME_WIDTH,
            height=CV_FRAME_HEIGHT,
            chn=CAM_CHN_ID_2,
        )
        sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_2)

        Display.init(
            Display.ST7701,
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            fps=60,
            to_ide=True,
        )
        display_ready = True

        encoder = Encoder()
        encoder.SetOutBufs(
            VENC_BUFFER_COUNT, video_width, video_height
        )
        encoder.Create(
            create_encoder_attr(encoder, video_width, video_height)
        )
        encoder_created = True

        sensor_source = sensor.bind_info(chn=CAM_CHN_ID_0)["src"]
        encoder_destination = (
            VIDEO_ENCODE_MOD_ID,
            VENC_DEV_ID,
            encoder.chn,
        )
        media_link = MediaManager.link(sensor_source, encoder_destination)
        MediaManager.init()
        media_ready = True

        rtsp = mm.rtsp_server()
        rtsp.rtspserver_init(RTSP_PORT)
        rtsp_ready = True
        rtsp.rtspserver_createsession(
            RTSP_SESSION,
            mm.multi_media_type.media_h264,
            False,
        )
        rtsp.rtspserver_start()
        rtsp_started = True

        encoder.Start()
        encoder_started = True
        sensor.run()
        touch = TOUCH(0)

        url = "rtsp://%s:%d/%s" % (ip, RTSP_PORT, RTSP_SESSION)
        print("")
        print("================================================")
        print("[MODE %d] local screen: CV overlays" % mode_id)
        print("[MODE %d] RTSP: raw camera image only" % mode_id)
        print("[RTSP] VLC address:", url)
        print("[CV] UART3 GPIO%d TX / GPIO%d RX" % (
            UART3_TX_PIN, UART3_RX_PIN))
        print("[CV] ROI y=%d..%d radius=%d..%d" % (
            roi_y0, roi_y1, MIN_RADIUS, MAX_RADIUS))
        print("================================================")

        stream_data = StreamData()
        frame_index = 0
        total_frames = 0
        period_frames = 0
        period_bytes = 0
        stats_started = time.ticks_ms()
        gc_started = stats_started
        next_uart_send_ms = stats_started
        mode_command_sent = False
        task_armed = False
        valid_packets = 0
        lost_packets = 0
        last_center_x = None
        last_display_best = None
        display_miss_frames = 0
        mode6_target_x = int(round(CALIBRATION_X_ZERO))
        mode6_target_x100 = 0
        mode6_target_dirty = False
        clock = time.clock()

        while True:
            os.exitpoint()

            # Service one raw H.264 frame. This channel is never modified.
            result = encoder.GetStream(stream_data)
            if result in (0, None):
                frame_bytes = 0
                frame_timestamp = time.ticks_ms()
                try:
                    for index in range(stream_data.pack_cnt):
                        packet_size = stream_data.data_size[index]
                        if packet_size <= 0:
                            continue
                        packet = bytes(uctypes.bytearray_at(
                            stream_data.data[index], packet_size
                        ))
                        rtsp.rtspserver_sendvideodata(
                            RTSP_SESSION,
                            packet,
                            packet_size,
                            frame_timestamp,
                        )
                        frame_bytes += packet_size
                finally:
                    encoder.ReleaseStream(stream_data)

                total_frames += 1
                period_frames += 1
                period_bytes += frame_bytes

            # CV reads only channel 2; overlays are applied after recognition.
            clock.tick()
            frame = sensor.snapshot(chn=CAM_CHN_ID_2)
            frame_array = frame.to_numpy_ref()
            best, candidates, fallback_used = find_best_circle(
                frame_array, roi_y0, roi_y1, last_center_x
            )
            fps = clock.fps()

            if touch is not None:
                points = touch.read(1)
                if len(points):
                    point = points[0]
                    arm_pressed = (
                        (not task_armed)
                        and point.event == TOUCH.EVENT_DOWN
                        and point_in_rect(point.x, point.y, TASK_ARM_RECT)
                    )
                    if arm_pressed:
                        task_armed = True
                        valid_packets = 0
                        print("[TASK] MODE %d armed by touch" % mode_id)
                    elif (mode_id == 6
                          and point.event in (
                              TOUCH.EVENT_DOWN, TOUCH.EVENT_MOVE)):
                        if point.y >= roi_y0 and point.y < roi_y1:
                            mode6_target_x = max(
                                MODE6_TARGET_X_MIN,
                                min(point.x, MODE6_TARGET_X_MAX),
                            )
                            mode6_target_x100 = int(round(
                                position_from_x(mode6_target_x) * 100.0
                            ))
                            mode6_target_dirty = True
                    elif mode_id == 6 and point.event == TOUCH.EVENT_UP:
                        if mode6_target_dirty and mode_command_sent:
                            uart.write((
                                "TARGET,%d\n" % mode6_target_x100
                            ).encode())
                            print(
                                "[UART] MODE6 target=%+.2f cm"
                                % (mode6_target_x100 / 100.0)
                            )
                        mode6_target_dirty = False

            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, next_uart_send_ms) >= 0:
                mode_command_sent, valid_packets, lost_packets = (
                    send_control_packet(
                        uart,
                        best,
                        mode_id,
                        mode_command_sent,
                        valid_packets,
                        lost_packets,
                        mode6_target_x100,
                        task_armed,
                    )
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
            else:
                display_miss_frames += 1
                if lost_packets >= NOBALL_CONFIRM_PACKETS:
                    last_center_x = None
                if (
                    last_display_best is not None
                    and display_miss_frames <= DISPLAY_HOLD_FRAMES
                ):
                    display_best = last_display_best
                    display_hold = True
                else:
                    display_best = None
                    display_hold = False

            draw_cv_results(
                frame,
                roi_y0,
                roi_y1,
                best,
                display_best,
                display_hold,
                candidates,
                fps,
                fallback_used,
                lost_packets,
                mode_id,
                mode6_target_x,
                mode6_target_x100,
            )
            draw_task_gate(frame, mode_id, task_armed)
            Display.show_image(frame)

            if frame_index % PRINT_EVERY_N_FRAMES == 0:
                if best is None:
                    print("[CV] circles=%d position=NA fps=%.1f" % (
                        len(candidates), fps))
                else:
                    print(
                        "[CV] circles=%d center=(%d,%d) r=%d "
                        "position=%+.2fcm fps=%.1f"
                        % (
                            len(candidates), best[0], best[1],
                            best[2], best[3], fps,
                        )
                    )
            frame_index += 1

            elapsed = time.ticks_diff(now_ms, stats_started)
            if elapsed >= STATS_PERIOD_MS:
                seconds = elapsed / 1000.0
                stream_fps = period_frames / seconds
                mbps = period_bytes * 8.0 / seconds / 1000000.0
                print("[RTSP] FPS: %.1f bitrate: %.2f Mbps frames: %d" % (
                    stream_fps, mbps, total_frames))
                period_frames = 0
                period_bytes = 0
                stats_started = now_ms

            if time.ticks_diff(now_ms, gc_started) >= GC_PERIOD_MS:
                gc.collect()
                gc_started = now_ms

    except KeyboardInterrupt:
        print("[MODE %d] stopped by IDE" % mode_id)
    except BaseException as error:
        print("[MODE %d] error:" % mode_id, repr(error))
        raise
    finally:
        print("[MODE %d] releasing resources" % mode_id)

        if touch is not None:
            try:
                touch.deinit()
            except BaseException as error:
                print("[MODE6] touch cleanup:", repr(error))

        if sensor is not None:
            try:
                sensor.stop()
            except BaseException as error:
                print("[CV] sensor cleanup:", repr(error))

        if encoder_started and encoder is not None:
            try:
                encoder.Stop()
            except BaseException as error:
                print("[RTSP] encoder stop:", repr(error))

        if media_link is not None:
            try:
                media_link.destroy()
            except BaseException:
                pass

        if encoder_created and encoder is not None:
            try:
                encoder.Destroy()
            except BaseException as error:
                print("[RTSP] encoder destroy:", repr(error))

        if rtsp_started and rtsp is not None:
            try:
                rtsp.rtspserver_stop()
            except BaseException as error:
                print("[RTSP] stop:", repr(error))

        if rtsp_ready and rtsp is not None:
            try:
                rtsp.rtspserver_destroysession(RTSP_SESSION)
            except BaseException:
                pass
            try:
                rtsp.rtspserver_deinit()
            except BaseException as error:
                print("[RTSP] deinit:", repr(error))

        if display_ready:
            try:
                Display.deinit()
            except BaseException as error:
                print("[CV] display cleanup:", repr(error))

        if media_ready:
            try:
                time.sleep_ms(100)
                MediaManager.deinit()
            except BaseException as error:
                print("[Media] cleanup:", repr(error))

        if uart is not None:
            try:
                uart.deinit()
            except BaseException as error:
                print("[CV] UART cleanup:", repr(error))

        if wlan is not None:
            try:
                wlan.stop()
            except BaseException as error:
                print("[RTSP] Wi-Fi cleanup:", repr(error))

        gc.collect()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        print("[MODE %d] finished" % mode_id)


def main():
    touch = None
    canvas = None
    selected_index = 0
    running_mode = None
    status_text = "Touch a mode button"
    last_touch = None
    redraw_needed = True
    launch_mode = None
    gc_started = time.ticks_ms()

    os.exitpoint(os.EXITPOINT_ENABLE)

    try:
        Display.init(
            Display.ST7701,
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            fps=60,
            to_ide=True,
        )
        touch = TOUCH(0)
        canvas = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)

        print("[MENU] touch menu ready: 800x480")
        print("[MENU] modes: 1, 2, 3, 4, 5, 6")

        while True:
            os.exitpoint()
            points = touch.read(1)
            if len(points):
                point = points[0]
                if point.event == TOUCH.EVENT_DOWN:
                    last_touch = (point.x, point.y)
                    selected_index, running_mode, status_text = handle_touch(
                        point.x,
                        point.y,
                        selected_index,
                        running_mode,
                    )
                    if (point_in_rect(point.x, point.y, START_RECT)
                            and running_mode in (1, 2, 3, 4, 5, 6)):
                        launch_mode = running_mode
                        if running_mode in (1, 2):
                            status_text = "Starting raw RTSP; reset to return"
                        else:
                            status_text = "Opening RTSP; task remains stopped"
                    redraw_needed = True

            if redraw_needed:
                draw_menu(
                    canvas,
                    selected_index,
                    running_mode,
                    status_text,
                    last_touch,
                )
                Display.show_image(canvas)
                redraw_needed = False

            if launch_mode in (1, 2, 3, 4, 5, 6):
                time.sleep_ms(500)
                break

            now = time.ticks_ms()
            if time.ticks_diff(now, gc_started) >= 5000:
                gc.collect()
                gc_started = now

            time.sleep_ms(10)

    except KeyboardInterrupt:
        print("[MENU] IDE interrupt")
    except BaseException as error:
        print_exception_compatible(error)
    finally:
        if touch is not None:
            try:
                touch.deinit()
            except BaseException as error:
                print("[MENU] touch deinit:", repr(error))
        try:
            Display.deinit()
        except BaseException as error:
            print("[MENU] display deinit:", repr(error))

        touch = None
        canvas = None
        gc.collect()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(300)
        print("[MENU] stopped")

    if launch_mode in (1, 2):
        gc.collect()
        run_raw_rtsp(launch_mode)
    elif launch_mode in (3, 4, 5, 6):
        gc.collect()
        run_ball_mode_with_raw_rtsp(launch_mode)


if __name__ == "__main__":
    main()
