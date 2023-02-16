# Declare a cc_library rule to build the Zed SDK library.
cc_library(
    name = "zed-core",
    srcs = glob(["/usr/local/zed/include/**/*.hpp"], exclude=["/usr/local/zed/include/sl/**/sample/**"]),
    hdrs = glob(["/usr/local/zed/include/**/*.hpp"], exclude=["/usr/local/zed/include/sl/**/sample/**"]),
    includes = [        "/usr/local/zed/include",    ],
    linkopts = [        
        "-lstdc++",        
        "-lm",        
        "-lX11",        
        "-lXext",        
        "-lGL",        
        "-ldl",        
        "-lglfw3",        
        "-lz",    
    ],
)

# Declare a native library rule to load the Zed SDK library.
native.cc_library(
    name = "zed-cuda",
    srcs = glob(["/usr/local/zed/lib/libsl_*.so"]),
    linkopts = [        "-L/usr/local/zed/lib",    ],
)