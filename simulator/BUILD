load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")

cc_binary(
    name = "main",
    srcs = ["main.cc"],
    deps = [
        ":console",
        ":hooks",
        ":mouse_cursor_icon",
        "//tplp/config:params",
        "//tplp/ui:main",
        "@FreeRTOS",
        "@SDL2",
        "@lvgl",
    ],
)

cc_library(
    name = "hooks",
    srcs = ["hooks.cc"],
    hdrs = ["hooks.h"],
    deps = [
        ":console",
        "@FreeRTOS",
    ],
)

cc_library(
    name = "console",
    srcs = ["console.cc"],
    hdrs = ["console.h"],
    deps = [
        "@FreeRTOS",
    ],
)

cc_library(
    name = "mouse_cursor_icon",
    srcs = ["mouse_cursor_icon.c"],
    deps = [
        "@lvgl",
    ],
)

# Please see the following issue if there's trouble with this...
# https://github.com/hedronvision/bazel-compile-commands-extractor/issues/70
refresh_compile_commands(
    name = "refresh_compile_commands",
    targets = {
        "//simulator/...": "",
    },
)
