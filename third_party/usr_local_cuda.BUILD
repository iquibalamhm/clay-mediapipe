# third_party/usr_local.BUILD
cc_library(
    name = "my_cuda_library",
    #srcs = ["my_cuda_file.cu"],
    #hdrs = ["my_cuda_file.h"],
    #copts = ["-Iinclude"],
    hdrs = glob(["include/**"]),
    srcs = [
        "lib64/libcudart.so",
    ],
    includes = [
        "include",
    ],
    linkopts = [
        "-Llib64",
        "-lGL",
        "-lcudart",
        "-lGLEW",
        "-lX11",
        "-lXi",
        "-lXrandr",
        "-lXinerama",
        "-lXcursor",
        "-lrt",
        "-lm",
        "-lglut",
        "-lGLU",
    ],
    #linkopts = ["-Llib64", "-lcudart"],
    visibility = ["//visibility:public"],
    #cuda = True,
)