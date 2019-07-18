gst-launch-1.0 videomixer name=mix ! nvvidconv ! \
omxh265enc control-rate=2 bitrate=2000 ! 'video/x-h265, stream-format=(string)byte-stream' ! h265parse ! rtph265pay mtu=1400 ! udpsink host=10.42.0.1 port=5000 sync=false async=false \
v4l2src device="/dev/video0" ! \
video/x-raw, width=320, height=240 ! \
videobox border-alpha=0 top=-480 left=-320 ! mix. \
v4l2src device="/dev/video1" ! \
video/x-raw, width=320, height=240 !  \
videobox border-alpha=0 top=-480 left=0 ! mix. \
v4l2src device="/dev/video2" ! \
video/x-raw, width=640, height=480 !  \
videobox border-alpha=0 top=0 left=0 ! mix.
