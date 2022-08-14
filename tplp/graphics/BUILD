cc_library(
    name = "lvgl_init",
    srcs = ["lvgl_init.cc"],
    hdrs = ["lvgl_init.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":lvgl_mutex",
        ":mouse_cursor_icon",
        "//picolog",
        "//tplp/HX8357",
        "//tplp/HX8357:lvgl_driver",
        "//tplp/SharpLCD",
        "//tplp/SharpLCD:lvgl_driver",
        "//tplp/TSC2007",
        "//tplp/TSC2007:lvgl_driver",
        "//tplp/config:tplp_config",
        "@FreeRTOS",
        "@lvgl",
    ],
)

cc_library(
    name = "lvgl_mutex",
    srcs = ["lvgl_mutex.cc"],
    hdrs = ["lvgl_mutex.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//picolog",
        "@FreeRTOS",
    ],
)

cc_library(
    name = "mouse_cursor_icon",
    srcs = ["mouse_cursor_icon.c"],
    hdrs = ["mouse_cursor_icon.h"],
    deps = [
        "@lvgl",
    ],
)
