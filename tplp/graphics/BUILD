cc_library(
    name = "graphics",
    srcs = ["graphics.cc"],
    hdrs = ["graphics.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":lvgl_mutex",
        "//tplp:logging",
        "//tplp:tplp_config",
        "//tplp/SharpLCD",
        "//tplp/SharpLCD:lvgl_driver",
        "@FreeRTOS-Kernel//:FreeRTOS",
        "@lvgl",
    ],
)

cc_library(
    name = "lvgl_mutex",
    srcs = ["lvgl_mutex.cc"],
    hdrs = ["lvgl_mutex.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@FreeRTOS-Kernel//:FreeRTOS",
    ],
)
