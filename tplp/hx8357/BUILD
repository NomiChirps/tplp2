cc_library(
    name = "hx8357",
    srcs = ["hx8357.cc"],
    hdrs = ["hx8357.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//tplp:rtos_util",
        "//tplp/bus:spi",
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
        ":hx8357",
        "//picolog",
        "//tplp/graphics:lvgl_mutex",
        "@lvgl",
    ],
)
