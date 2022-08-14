cc_library(
    name = "TSC2007",
    srcs = ["TSC2007.cc"],
    hdrs = ["TSC2007.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//picolog",
        "//picolog:status_macros",
        "//tplp:I2cController",
        "//tplp/config:tplp_config",
        "@FreeRTOS",
        "@rules_pico//pico:hardware_gpio",
        "@rules_pico//pico:hardware_irq",
    ],
)
