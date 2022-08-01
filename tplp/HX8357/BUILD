cc_library(
    name = "HX8357",
    srcs = ["HX8357.cc"],
    hdrs = ["HX8357.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//tplp:SpiManager",
        "//tplp:types",
        "@FreeRTOS",
    ],
)

cc_library(
    name = "lvgl_driver",
    srcs = ["lvgl_driver.cc"],
    hdrs = ["lvgl_driver.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":HX8357",
        "@lvgl",
    ],
)
