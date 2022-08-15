cc_library(
    name = "TSC2007",
    srcs = ["TSC2007.cc"],
    hdrs = ["TSC2007.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//picolog",
        "//picolog:status_macros",
        "//tplp/bus:I2cController",
        "@FreeRTOS",
        "@rules_pico//pico:hardware_gpio",
        "@rules_pico//pico:hardware_irq",
    ],
)

cc_library(
    name = "lvgl_driver",
    srcs = ["lvgl_driver.cc"],
    hdrs = ["lvgl_driver.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":TSC2007",
        "//picolog",
        "//tplp/graphics:lvgl_mutex",
        "@lvgl",
    ],
)
