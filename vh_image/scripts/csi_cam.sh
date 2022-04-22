gst-launch-1.0 nvarguscamerasrc  ! \
    video/x-raw,width=1280,height=720,framerate=30/1 ! \
    videoconvert ! \
    video/x-raw,format=YUY2 ! \
    nvjpegnc ! \
    image/jpeg,width=1920,height=1080,framerate=30/1 ! \
    shmsink enable-last-sample=false socket-path=/tmp/livesrc_XIASHI_L wait-for-connection=false
