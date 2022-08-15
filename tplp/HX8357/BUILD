cc_library(
    name = "HX8357",
    srcs = ["HX8357.cc"],
    hdrs = ["HX8357.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//tplp/bus:SpiController",
        "//tplp:rtos_util",
        "//tplp/bus:types",
        "@FreeRTOS",
        "@rules_pico//pico:hardware_gpio",
    ],
)

cc_library(
    name = "lvgl_driver",
    srcs = ["lvgl_driver.cc"],
    hdrs = ["lvgl_driver.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":HX8357",
        "//picolog",
        "//tplp/graphics:lvgl_mutex",
        "@lvgl",
    ],
)
