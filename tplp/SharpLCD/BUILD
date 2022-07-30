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
        "//tplp:SpiManager",
        "//tplp:logging",
    ],
)

cc_library(
    name = "lvgl_driver",
    srcs = ["lvgl_driver.cc"],
    hdrs = ["lvgl_driver.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":SharpLCD",
        "//tplp:logging",
        "//tplp/graphics:lvgl_mutex",
        "@lvgl",
    ],
)
