cc_library(
    name = "main",
    srcs = ["main.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":startup",
        "//tplp/config:hw",
        "@FreeRTOS",
        "@rules_pico//pico:pico_stdlib",
    ],
)

cc_library(
    name = "startup",
    srcs = ["startup.cc"],
    hdrs = ["startup.h"],
    deps = [
        "@FreeRTOS",
        "@rules_pico//pico:hardware_i2c",
        "@rules_pico//pico:hardware_spi",
        "//picolog",
        "//picolog:status_macros",
        "//tplp:RuntimeStats",
        "//tplp:TplpInterfaceImpl",
        "//tplp/adafruit_seesaw",
        "//tplp/bus:i2c",
        "//tplp/bus:spi",
        "//tplp/config:hw",
        "//tplp/config:tasks",
        "//tplp/graphics:lvgl_init",
        "//tplp/hx711",
        "//tplp/hx8357",
        "//tplp/tsc2007",

        # TODO: maybe lvgl_init should have these instead
        "//tplp/graphics:lvgl_mutex",
        "//tplp/ui:main",
    ],
)
