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

package(default_visibility = [
    "//visibility:public",
])

cc_library(
    name = "simple_run_graph_main",
    srcs = ["simple_run_graph_main.cc"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/port:file_helpers",
        "//mediapipe/framework/port:map_util",
        "//mediapipe/framework/port:parse_text_proto",
        "//mediapipe/framework/port:ret_check",
        "//mediapipe/framework/port:status",
        "//mediapipe/framework/port:statusor",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "demo_run_graph_main",
    srcs = ["demo_run_graph_main.cc"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework/port:file_helpers",
        "//mediapipe/framework/port:opencv_highgui",
        "//mediapipe/framework/port:opencv_imgproc",
        "//mediapipe/framework/port:opencv_video",
        "//mediapipe/framework/port:parse_text_proto",
        "//mediapipe/framework/port:status",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
    ],
)

# Linux only.
# Must have a GPU with EGL support:
# ex: sudo apt-get install mesa-common-dev libegl1-mesa-dev libgles2-mesa-dev
# (or similar nvidia/amd equivalent)
cc_library(
    name = "demo_run_graph_main_gpu",
    srcs = ["demo_run_graph_main_gpu.cc"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework/port:file_helpers",
        "//mediapipe/framework/port:opencv_highgui",
        "//mediapipe/framework/port:opencv_imgproc",
        "//mediapipe/framework/port:opencv_video",
        "//mediapipe/framework/port:parse_text_proto",
        "//mediapipe/framework/port:status",
        "//mediapipe/gpu:gl_calculator_helper",
        "//mediapipe/gpu:gpu_buffer",
        "//mediapipe/gpu:gpu_shared_data_internal",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
    ],
)

cc_library(
    name = "multi_hand_tracking_landmarks_gpu",
    srcs = ["multi_hand_tracking_landmarks_gpu.cc"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "//mediapipe/framework/formats:detection_cc_proto",
        "//mediapipe/framework/formats:rect_cc_proto",
        "//mediapipe/framework/port:file_helpers",
        "//mediapipe/framework/port:opencv_highgui",
        "//mediapipe/framework/port:opencv_imgproc",
        "//mediapipe/framework/port:opencv_video",
        "//mediapipe/framework/port:parse_text_proto",
        "//mediapipe/framework/port:status",
        "//mediapipe/gpu:gl_calculator_helper",
        "//mediapipe/gpu:gpu_buffer",
        "//mediapipe/gpu:gpu_shared_data_internal",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
    ],
)

cc_library(
    name = "mode",
    srcs = [
        "Mode.cpp",
        "PlayMode.cpp",
        "DrawLines.cpp",
        "PathFont.cpp",
        "PathFont-font.cpp",
        "ColorProgram.cpp",
        "GL.cpp",
        "Load.cpp",
        "gl_compile_program.cpp",
        "load_save_png.cpp",
        "data_path.cpp",
        "LitColorTextureProgram.cpp"
    ],
    hdrs = [
        "Mode.hpp",
        "PlayMode.hpp",
        "DrawLines.hpp",
        "PathFont.hpp",
        "ColorProgram.hpp",
        "GL.hpp",
        "Load.hpp",
        "gl_compile_program.hpp",
        "gl_errors.hpp",
        "load_save_png.hpp",
        "data_path.hpp",
        "LitColorTextureProgram.hpp"
    ],
    deps = [
        "//nest-libs:SDL",
        "//nest-libs:glm"
    ],
)
cc_library(
    name = "GLViewer",
    srcs = ["GLViewer.cpp"],
    hdrs = ["GLViewer.hpp"],
    deps = [
    "@usr_local_zed//:zed",
    "@usr_local_cuda//:my_cuda_library",
    ],
)



cc_library(
    name = "main",
    srcs = ["main.cpp"],
    deps = [
        ":mode",
        "//nest-libs:SDL",
        "//nest-libs:glm",
        "//nest-libs:zlib",
        "//nest-libs:png",
    ],
)

cc_library(
    name = "main_mediapipe",
    srcs = ["main_mediapipe.cpp"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "//mediapipe/framework/formats:detection_cc_proto",
        "//mediapipe/framework/formats:rect_cc_proto",
        "//mediapipe/framework/port:file_helpers",
        "//mediapipe/framework/port:opencv_highgui",
        "//mediapipe/framework/port:opencv_imgproc",
        "//mediapipe/framework/port:opencv_video",
        "//mediapipe/framework/port:parse_text_proto",
        "//mediapipe/framework/port:status",
        "//mediapipe/gpu:gl_calculator_helper",
        "//mediapipe/gpu:gpu_buffer",
        "//mediapipe/gpu:gpu_shared_data_internal",
        ":mode",
        "//nest-libs:SDL",
        "//nest-libs:glm",
        "//nest-libs:zlib",
        "//nest-libs:png",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
    ],
)
cc_library(
    name = "share_memory",
    srcs = ["sharing_memory.cpp"],
    hdrs = [
        "doc_anonymous_semaphore_shared_data.hpp"
    ],
)
cc_library(
    name = "read_memory",
    srcs = ["reading_memory.cpp"],
    hdrs = [
        "doc_anonymous_semaphore_shared_data.hpp"
    ],
)

cc_library(
    name = "read_memory_vector",
    srcs = ["reading_memory_vector.cpp"],
    hdrs = [
        "doc_anonymous_semaphore_shared_data.hpp"
    ],
)
cc_library(
    name = "share_memory_vector",
    srcs = ["sharing_memory_vector.cpp"],
    hdrs = [
        "doc_anonymous_semaphore_shared_data.hpp"
    ],
)

cc_library(
    name = "zed_lib",
    srcs = ["GLViewer.cpp"],
    hdrs = ["GLViewer.hpp"],
)

cc_library(
    name = "zed_depth",
    srcs = ["main_zed.cpp"],
    deps = [
        ":GLViewer",
        "@usr_local_zed//:zed",
        "@usr_local_cuda//:my_cuda_library",
    ],
)