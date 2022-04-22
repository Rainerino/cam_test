gst-launch-1.0 v4l2src device=/dev/video2 ! \
    image/jpeg,width=1280,height=720,framerate=60/1 ! \
    watchdog name=jpeg timeout=10000 ! \
    shmsink enable-last-sample=false socket-path=/tmp/livesrc_XIASHI_L wait-for-connection=false
