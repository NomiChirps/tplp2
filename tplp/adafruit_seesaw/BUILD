cc_library(
    name = "adafruit_seesaw",
    srcs = [
        "adafruit_seesaw.cc",
        "defs.h",
    ],
    hdrs = ["adafruit_seesaw.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//picolog",
        "//picolog:status",
        "//picolog:status_macros",
        "//picolog:statusor",
        "//tplp/bus:i2c",
    ],
)

cc_library(
    name = "4991_lvgl_driver",
    srcs = ["4991_lvgl_driver.cc"],
    hdrs = ["4991_lvgl_driver.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":adafruit_seesaw",
        "//picolog",
        "@lvgl",
    ],
)
