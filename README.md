# ROS2 OWO GStreamer UWU

## Build 
> default to /dev/video0, change it in /config/cam_param.yaml. Run colcon build again after changing to make sure it's loaded

```bash
# create workspace etc...
colcon build  --symlink-install --packages-select vh_image   
source install/local_setup.zsh  
```

## Run 


### ROS2 -> GStreamer

```bash
ros2 launch vh_image ros2_cam.launch.py   
```

### GStreamer -> ROS2

> default to /dev/video2
in terminal 1
```bash
./script/usb_cam.sh
```

in termainl 2
```bash
ros2 run vh_image vh_image    
```