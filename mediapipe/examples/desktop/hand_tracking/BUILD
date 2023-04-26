# Copyright 2019 The MediaPipe Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

cc_binary(
    name = "hand_tracking_tflite",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:simple_run_graph_main",
        "//mediapipe/graphs/hand_tracking:desktop_tflite_calculators",
    ],
)

cc_binary(
    name = "hand_tracking_cpu",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:demo_run_graph_main",
        "//mediapipe/graphs/hand_tracking:desktop_tflite_calculators",
    ],
)

# Linux only
cc_binary(
    name = "hand_tracking_gpu",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:demo_run_graph_main_gpu",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)

cc_binary(
    name = "hand_tracking_cpu_fps",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:demo_run_graph_main_fps",
        "//mediapipe/graphs/hand_tracking:desktop_tflite_calculators",
    ],
)

# Linux only
cc_binary(
    name = "hand_tracking_gpu_fps",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:demo_run_graph_main_gpu_fps",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)


# Linux only
cc_binary(
    name = "multi_hand_tracking_landmarks_gpu",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:multi_hand_tracking_landmarks_gpu",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)

# Linux only
cc_binary(
    name = "hand_tracking_gpu_clay",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:game",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)
# Linux game for playing with clay
cc_binary(
    name = "main_game",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:main_game",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)

cc_binary(
    name = "game",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:game",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)

# Linux game for playing with clay
cc_binary(
    name = "game_mediapipe",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:main_mediapipe",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)

# ZED camera depth sensing
cc_binary(
    name = "zed_depth_sensing",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:zed_depth",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)

# ZED camera depth sensing
cc_binary(
    name = "zed_depth_color",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:zed_depth_color",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)

# ZED camera depth sensing
cc_binary(
    name = "zed_depth_color_calibration",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:zed_depth_color_calibration",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)

# ZED camera depth sensing
cc_binary(
    name = "simulation_test",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:simulation_test",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)

# Fake vector generator
cc_binary(
    name = "fake_vector",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:fake_vector",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)

# Fake vector generator
cc_binary(
    name = "serial_communication",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:serial_communication",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)

# Fake vector generator
cc_binary(
    name = "write_memory",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:write_memory",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)

# Fake vector generator
cc_binary(
    name = "read_memory",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:read_memory",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)

# Fake vector generator
cc_binary(
    name = "color_trackbar",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop:color_trackbar",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)
