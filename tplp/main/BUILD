cc_library(
    name = "main",
    srcs = ["main.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":startup",
        "@FreeRTOS",
    ],
)

cc_library(
    name = "startup",
    srcs = ["startup.cc"],
    hdrs = ["startup.h"],
    deps = [
        "//tplp/config:pins",
        "//tplp/config:tasks",
        "@FreeRTOS",
        "//picolog",
        "//tplp/HX8357",
        "//tplp:I2cController",
        "//tplp:RuntimeStats",
        "//tplp:SpiController",
        "//tplp/TSC2007",
        "//tplp:TplpInterfaceImpl",
        "//tplp/graphics:lvgl_init",
        "@rules_pico//pico:hardware_spi",
        "@rules_pico//pico:hardware_i2c",

        # hmm, maybe lvgl_init should have these instead
        "//tplp/graphics:lvgl_mutex",
        "//tplp/ui:main",
    ],
)
