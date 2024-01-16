## Build and run game interface
```

azel build -c opt --copt -DMESA_EGL_NO_X11_HEADERS --copt -DEGL_NO_X11   mediapipe/examples/desktop/hand_tracking:main_game

GLOG_logtostderr=1 bazel-bin/mediapipe/examples/desktop/hand_tracking/main_game /dev/ttyACM1

```

## Build and run hand/color detection
```
bazel build -c opt --copt -DMESA_EGL_NO_X11_HEADERS --copt -DEGL_NO_X11   mediapipe/examples/desktop/hand_tracking:zed_depth_color_calibration

GLOG_logtostderr=1 bazel-bin/mediapipe/examples/desktop/hand_tracking/zed_depth_color_calibration VGA zed color --calculator_graph_config_file=mediapipe/graphs/hand_tracking/hand_tracking_desktop_live_gpu.pbtxt

```
