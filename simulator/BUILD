load("@lvgl//:defs.bzl", "lvgl_configured_library")

lvgl_configured_library(
    name = "lvgl",
    lv_conf = "//simulator/config:lv_conf",
    visibility = ["//simulator:__subpackages__"],
)

cc_binary(
    name = "main",
    srcs = ["main.c"],
    deps = [
        "//simulator:lvgl",
        "//simulator/ui:main",
        # TODO: sdl2
        # TODO: lv_drivers
    ],
)
