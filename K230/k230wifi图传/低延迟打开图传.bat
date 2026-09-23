@echo off
start "" "C:\Program Files (x86)\VideoLAN\VLC\vlc.exe" ^
  --network-caching=150 ^
  --live-caching=150 ^
  --clock-jitter=0 ^
  --clock-synchro=0 ^
  --video-filter=transform ^
  --transform-type=270 ^
  "rtsp://192.168.4.1:8554/live"
