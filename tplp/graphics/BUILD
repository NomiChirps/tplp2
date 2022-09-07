cc_library(
    name = "lvgl_init",
    srcs = ["lvgl_init.cc"],
    hdrs = ["lvgl_init.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":lvgl_mutex",
        ":mouse_cursor_icon",
        "//picolog",
        "//tplp/hx8357",
        "//tplp/hx8357:lvgl_driver",
        "//tplp/SharpLCD",
        "//tplp/SharpLCD:lvgl_driver",
        "//tplp/tsc2007",
        "//tplp/tsc2007:tsc2007_lvgl_driver",
        "//tplp/adafruit_seesaw",
        "//tplp/adafruit_seesaw:4991_lvgl_driver",
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
