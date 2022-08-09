load("@lvgl//:defs.bzl", "lvgl_configured_library")
load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")

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

# For VSCode
refresh_compile_commands(
    name = "refresh_compile_commands",
    targets = {
        # In the absence of an explicit `-x` flag specifying the language,
        # clang/clangd uses the file extension to guess. In the case of C++
        # header files named "foo.h", it guesses C, and doesn't include the
        # C++ system headers.
        # Actually, it could be a totally different issue. See:
        # https://github.com/hedronvision/bazel-compile-commands-extractor/issues/70
        "//simulator/...": "--cxxopt=-xc++",
    },
)
