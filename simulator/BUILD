load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")

cc_binary(
    name = "main",
    srcs = ["main.cc"],
    deps = [
        ":console",
        ":hooks",
        ":mouse_cursor_icon",
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
