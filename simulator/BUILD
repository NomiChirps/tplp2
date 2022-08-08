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
        "//simulator:lvgl",
        "//simulator/ui:main",
        "@SDL2",
        ":mouse_cursor_icon",
        # TODO: lv_drivers
    ],
)

cc_library(
    name = "mouse_cursor_icon",
    srcs = ["mouse_cursor_icon.c"],
    deps = [
        "//simulator:lvgl",
    ],
)
