cc_library(
    name = "SharpLCD",
    srcs = [
        "SharpLCD.cc",
    ],
    hdrs = [
        "SharpLCD.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//picolog",
        "//tplp:SpiManager",
        "//tplp:tplp_config",
    ],
)

cc_library(
    name = "lvgl_driver",
    srcs = ["lvgl_driver.cc"],
    hdrs = ["lvgl_driver.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":SharpLCD",
        "//lib:lvgl",
        "//picolog",
        "//tplp:tplp_config",
        "//tplp/graphics:lvgl_mutex",
    ],
)
