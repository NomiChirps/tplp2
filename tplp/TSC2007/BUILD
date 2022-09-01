cc_library(
    name = "TSC2007",
    srcs = ["TSC2007.cc"],
    hdrs = ["TSC2007.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//picolog",
        "//picolog:status_macros",
        "//tplp/bus:i2c",
        "@FreeRTOS",
        "@rules_pico//pico:hardware_gpio",
        "@rules_pico//pico:hardware_irq",
    ],
)

cc_library(
    name = "tsc2007_lvgl_driver",
    srcs = ["tsc2007_lvgl_driver.cc"],
    hdrs = ["tsc2007_lvgl_driver.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":TSC2007",
        "//picolog",
        "//tplp/graphics:lvgl_mutex",
        "@lvgl",
    ],
)
