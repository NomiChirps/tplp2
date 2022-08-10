cc_library(
    name = "lv_conf",
    hdrs = [
        "lv_conf.h",
    ],
    strip_include_prefix = "/simulator/config",
    visibility = ["//simulator:__subpackages__"],
    deps = [
        "@SDL2//:headers_only",
    ],
)

cc_library(
    name = "lv_drv_conf",
    hdrs = [
        "lv_drv_conf.h",
    ],
    strip_include_prefix = "/simulator/config",
    visibility = ["//simulator:__subpackages__"],
    deps = [
        "@SDL2//:headers_only",
    ],
)

cc_library(
    name = "FreeRTOSConfig",
    hdrs = ["FreeRTOSConfig.h"],
    strip_include_prefix = "/simulator/config",
    visibility = ["//visibility:public"],
)
