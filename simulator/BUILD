load("@lvgl//:defs.bzl", "lvgl_configured_library")

lvgl_configured_library(
    name = "lvgl",
    lv_conf = "//simulator/config:lv_conf",
    lv_drv_conf = "//simulator/config:lv_drv_conf",
    visibility = ["//simulator:__subpackages__"],
)

cc_binary(
    name = "main",
    srcs = ["main.c"],
    deps = [
        ":console",
        ":hooks",
        ":mouse_cursor_icon",
        "//simulator:lvgl",
        "//simulator/ui:main",
        "@FreeRTOS",
        "@SDL2",
    ],
)

cc_library(
    name = "hooks",
    srcs = ["hooks.c"],
    hdrs = ["hooks.h"],
    deps = [
        ":console",
        "@FreeRTOS",
    ],
)

cc_library(
    name = "console",
    srcs = ["console.c"],
    hdrs = ["console.h"],
    deps = [
        "@FreeRTOS",
    ],
)

cc_library(
    name = "mouse_cursor_icon",
    srcs = ["mouse_cursor_icon.c"],
    deps = [
        "//simulator:lvgl",
    ],
)

cc_library(
    name = "FreeRTOSConfig",
    hdrs = ["FreeRTOSConfig.h"],
    strip_include_prefix = "/simulator",
    visibility = ["//visibility:public"],
)
