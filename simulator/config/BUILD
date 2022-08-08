cc_library(
    name = "lv_conf",
    hdrs = [
        "lv_conf.h",
    ],
    strip_include_prefix = "/simulator/config",
    visibility = ["//simulator:__subpackages__"],
)

cc_library(
    name = "lv_drv_conf",
    hdrs = [
        "lv_drv_conf.h",
    ],
    strip_include_prefix = "/simulator/config",
    visibility = ["//simulator:__subpackages__"],
    deps = [
        "@SDL2",
    ],
)
