cc_library(
    name = "tsc2007",
    srcs = ["tsc2007.cc"],
    hdrs = ["tsc2007.h"],
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
        ":tsc2007",
        "//picolog",
        "//tplp/config:constants",
        "//tplp/graphics:lvgl_mutex",
        "@lvgl",
    ],
)
