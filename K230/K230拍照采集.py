"""K230/CanMV 自动拍照采集程序。

使用方法：
1. 把本文件复制到 TF 卡后在 CanMV IDE 中运行。
2. 采集手拿钢球时，把 CAPTURE_MODE 设为 "positive"。
3. 采集电脑图标、鼠标、螺丝和反光物等误检画面时，确保画面中没有
   钢球，并把 CAPTURE_MODE 设为 "negative"。
4. 图片保存在 /sdcard/steel_ball_capture/。

本程序按固定时间间隔自动拍照，不依赖不同开发板上的按键引脚。
"""

from libs.PipeLine import PipeLine
from media.sensor import *
import cv_lite
import gc
import os
import time
import ulab.numpy as np


# 与当前 LushanPi Lite / K230D 推理代码保持一致。
SENSOR_ID = 2

# 每次运行前二选一："positive" 或 "negative"。
CAPTURE_MODE = "positive"

# Reuse PipeLine's existing 640x360 RGBP888 AI channel for saving. No extra
# sensor channel or rotate buffer is allocated.
CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 360
CAPTURE_INTERVAL_MS = 800
MAX_IMAGES = 300

ROOT_DIR = "/sdcard/steel_ball_capture"


def mkdir_p(path):
    current = ""
    for part in path.strip("/").split("/"):
        current += "/" + part
        try:
            os.mkdir(current)
        except OSError as error:
            # EEXIST
            if len(error.args) > 0 and error.args[0] == 17:
                pass
            else:
                try:
                    os.stat(current)
                except OSError:
                    raise


def image_dir():
    return ROOT_DIR + "/" + CAPTURE_MODE + "/images"


def label_dir():
    return ROOT_DIR + "/" + CAPTURE_MODE + "/labels"


def find_next_index():
    prefix = "pos_" if CAPTURE_MODE == "positive" else "neg_"
    next_index = 0
    try:
        names = os.listdir(image_dir())
    except OSError:
        return 0

    for name in names:
        if not name.startswith(prefix) or not name.endswith(".jpg"):
            continue
        try:
            value = int(name[len(prefix):-4])
            if value >= next_index:
                next_index = value + 1
        except ValueError:
            pass
    return next_index


def rgbp_to_rgb888(frame_array, rgb888):
    """Convert PipeLine's planar RGB buffer (CHW/NCHW) to RGB888 HWC."""
    shape = frame_array.shape

    # PipeLine normally returns (1, 3, H, W). Some firmware returns (3, H, W).
    if len(shape) == 4:
        if shape[0] != 1:
            raise ValueError("unexpected RGBP batch shape: %s" % (shape,))
        frame_array = frame_array[0]
        shape = frame_array.shape

    if len(shape) != 3:
        raise ValueError("unexpected camera frame shape: %s" % (shape,))

    if shape[0] == 3:
        # RGBP888 is planar: RRR..., GGG..., BBB...
        # Image encoders need interleaved RGBRGB... bytes. Older CanMV ulab
        # builds do not provide np.transpose(), so copy each plane separately.
        rgb888[:, :, 0] = frame_array[0, :, :]
        rgb888[:, :, 1] = frame_array[1, :, :]
        rgb888[:, :, 2] = frame_array[2, :, :]
    elif shape[2] == 3:
        # Compatibility path for firmware that already returns HWC.
        rgb888[:, :, :] = frame_array[:, :, :]
    else:
        raise ValueError("camera frame has no RGB channel axis: %s" % (shape,))

    if rgb888.shape[0] != CAPTURE_HEIGHT or rgb888.shape[1] != CAPTURE_WIDTH:
        raise ValueError(
            "camera frame size mismatch: %s, expected (%d, %d, 3)"
            % (rgb888.shape, CAPTURE_HEIGHT, CAPTURE_WIDTH)
        )
    return rgb888


def save_capture(frame_array, rgb888, index):
    prefix = "pos_" if CAPTURE_MODE == "positive" else "neg_"
    stem = prefix + ("%06d" % index)
    path = image_dir() + "/" + stem + ".jpg"

    # Never pass an RGBP/CHW frame directly to JPEG. It creates the coloured
    # stripe pattern seen in the previous captures. Convert to HWC first.
    rgbp_to_rgb888(frame_array, rgb888)
    cv_lite.save_image(
        path,
        [CAPTURE_HEIGHT, CAPTURE_WIDTH],
        rgb888,
    )
    file_size = os.stat(path)[6]
    if file_size < 1024:
        raise OSError("JPEG save failed or file is too small: " + path)

    # Negative/background images use empty YOLO labels.
    if CAPTURE_MODE == "negative":
        label_path = label_dir() + "/" + stem + ".txt"
        handle = open(label_path, "w")
        handle.close()

    print("[capture] saved:", path, "bytes:", file_size)
    gc.collect()
    return stem


def main():
    if CAPTURE_MODE not in ("positive", "negative"):
        raise ValueError('CAPTURE_MODE must be "positive" or "negative"')

    pipeline = None
    sensor = None
    saved_count = 0
    last_saved_name = ""

    mkdir_p(image_dir())
    if CAPTURE_MODE == "negative":
        mkdir_p(label_dir())
    next_index = find_next_index()

    try:
        # Use the exact same display path as the working recognition main.py.
        pipeline = PipeLine(
            rgb888p_size=[640, 360],
            display_mode="lcd",
            display_size=None,
            osd_layer_num=1,
            debug_mode=0,
        )
        sensor = Sensor(id=SENSOR_ID)
        pipeline.create(
            sensor=sensor,
            hmirror=False,
            vflip=False,
            fps=30,
            to_ide=True,
        )

        # Let exposure and white balance settle.
        for _ in range(30):
            frame_array = pipeline.get_frame()

        print("[capture] camera array shape:", frame_array.shape)
        if len(frame_array.shape) == 4:
            expected_shape = (
                frame_array.shape[0] == 1
                and frame_array.shape[1] == 3
                and frame_array.shape[2] == CAPTURE_HEIGHT
                and frame_array.shape[3] == CAPTURE_WIDTH
            )
        else:
            expected_shape = (
                len(frame_array.shape) == 3
                and (
                    (
                        frame_array.shape[0] == 3
                        and frame_array.shape[1] == CAPTURE_HEIGHT
                        and frame_array.shape[2] == CAPTURE_WIDTH
                    )
                    or (
                        frame_array.shape[0] == CAPTURE_HEIGHT
                        and frame_array.shape[1] == CAPTURE_WIDTH
                        and frame_array.shape[2] == 3
                    )
                )
            )
        if not expected_shape:
            raise ValueError(
                "unexpected PipeLine frame shape: %s" % (frame_array.shape,)
            )

        # Allocate once and reuse it. This avoids a large allocation on every
        # photo and is friendlier to K230's media memory.
        rgb888 = np.zeros(
            (CAPTURE_HEIGHT, CAPTURE_WIDTH, 3),
            dtype=np.uint8,
        )

        last_capture_ms = time.ticks_ms() - CAPTURE_INTERVAL_MS
        print("[capture] mode:", CAPTURE_MODE)
        print("[capture] output:", image_dir())

        while saved_count < MAX_IMAGES:
            # Required for responsive Stop in CanMV IDE.
            os.exitpoint()
            frame_array = pipeline.get_frame()
            now_ms = time.ticks_ms()

            if time.ticks_diff(now_ms, last_capture_ms) >= CAPTURE_INTERVAL_MS:
                last_saved_name = save_capture(frame_array, rgb888, next_index)
                next_index += 1
                saved_count += 1
                last_capture_ms = now_ms

            pipeline.osd_img.clear()
            pipeline.osd_img.draw_string_advanced(
                8,
                8,
                24,
                "mode:%s count:%d/%d"
                % (CAPTURE_MODE, saved_count, MAX_IMAGES),
                color=(0, 255, 0),
            )
            if last_saved_name:
                pipeline.osd_img.draw_string_advanced(
                    8,
                    40,
                    20,
                    "saved:" + last_saved_name,
                    color=(0, 255, 255),
                )
            pipeline.show_image()

        print("[capture] completed, saved:", saved_count)

    except KeyboardInterrupt:
        print("[capture] stopped, saved:", saved_count)
    except BaseException as error:
        import sys
        sys.print_exception(error)
    finally:
        if pipeline is not None and getattr(pipeline, "sensor", None) is not None:
            try:
                pipeline.destroy()
            except BaseException:
                pass
        time.sleep_ms(100)
        gc.collect()


main()
