"""
LCKFB Lushan Pi K230: camera -> hardware H.264 -> Wi-Fi RTSP.

This is the standalone video-transmission task. It does not load YOLO/KModel.

K230 hotspot:
  SSID: K230-Video
  password: 12345678

VLC address (normally):
  rtsp://192.168.4.1:8554/live
"""

import gc
import network
import os
import time
import uctypes
import multimedia as mm

from media.sensor import *
from media.vencoder import *
from media.media import *


# Standard Lushan Pi K230 default CSI camera used by the working local program.
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


def start_hotspot():
    wlan = network.WLAN(network.AP_IF)
    if not wlan.active():
        wlan.active(True)

    # The short config form may make an 8-character key advertise as legacy
    # WEP.  Modern phones and Windows reject WEP, so restart it explicitly as
    # WPA2-AES on the board's 2.4 GHz radio.
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
    print("[WiFi] AP info:", wlan.info())
    print("[WiFi] network:", wlan.ifconfig())
    print("========================================")
    return wlan, ip


def create_encoder_attr(encoder, width, height):
    # Newer legacy firmware accepts bitrate/GOP. The four-argument fallback is
    # compatible with older CanMV firmware and uses the firmware defaults.
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
        print("[Video] encoder parameters: %d kbps, GOP %d" % (
            VIDEO_BITRATE_KBPS, VIDEO_GOP))
        return attr
    except TypeError:
        print("[Video] old ChnAttrStr API; using firmware bitrate defaults")
        return ChnAttrStr(
            encoder.PAYLOAD_TYPE_H264,
            encoder.H264_PROFILE_BASELINE,
            width,
            height,
        )


def main():
    wlan = None
    sensor = None
    encoder = None
    media_link = None
    rtsp = None

    media_ready = False
    encoder_created = False
    encoder_started = False
    sensor_started = False
    rtsp_ready = False
    rtsp_started = False

    width = ALIGN_UP(VIDEO_WIDTH, 16)
    height = VIDEO_HEIGHT

    os.exitpoint(os.EXITPOINT_ENABLE)

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
        encoder.SetOutBufs(
            VENC_BUFFER_COUNT,
            width,
            height,
        )

        encoder_attr = create_encoder_attr(encoder, width, height)
        encoder.Create(encoder_attr)
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
        sensor_started = True

        url = "rtsp://%s:%d/%s" % (ip, RTSP_PORT, RTSP_SESSION)
        print("")
        print("================================================")
        print("[RTSP] camera stream ready")
        print("[RTSP] VLC address:", url)
        print("[RTSP] video: %dx%d @ %d FPS, H.264" % (
            width, height, SENSOR_FPS))
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
        print("[Main] stopped by user")
    except BaseException as error:
        print("[Main] error:")
        print(repr(error))
        raise
    finally:
        print("[Stop] releasing resources")

        # sensor.reset() already reserves the device.  Release it even when an
        # exception happens before sensor.run(), otherwise the next run fails
        # with "sensor(2) is already inited".
        if sensor is not None:
            try:
                sensor.stop()
            except BaseException as error:
                print("[Stop] sensor:", repr(error))

        if media_link is not None:
            try:
                media_link.destroy()
            except BaseException:
                pass
            media_link = None

        if encoder_started and encoder is not None:
            try:
                encoder.Stop()
            except BaseException as error:
                print("[Stop] encoder stop:", repr(error))

        if encoder_created and encoder is not None:
            try:
                encoder.Destroy()
            except BaseException as error:
                print("[Stop] encoder destroy:", repr(error))

        if rtsp_started and rtsp is not None:
            try:
                rtsp.rtspserver_stop()
            except BaseException as error:
                print("[Stop] RTSP stop:", repr(error))

        if rtsp_ready and rtsp is not None:
            try:
                rtsp.rtspserver_destroysession(RTSP_SESSION)
            except BaseException:
                pass
            try:
                rtsp.rtspserver_deinit()
            except BaseException as error:
                print("[Stop] RTSP deinit:", repr(error))

        if media_ready:
            try:
                time.sleep_ms(100)
                MediaManager.deinit()
            except BaseException as error:
                print("[Stop] media:", repr(error))

        if wlan is not None:
            try:
                wlan.stop()
            except BaseException as error:
                print("[Stop] WiFi:", repr(error))
        wlan = None
        gc.collect()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        print("[Main] finished")


if __name__ == "__main__":
    main()
