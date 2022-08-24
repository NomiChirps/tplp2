load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")

# Please see the following issue if there's trouble with this...
# https://github.com/hedronvision/bazel-compile-commands-extractor/issues/70
refresh_compile_commands(
    name = "refresh_compile_commands_pico",
    exclude_headers = "external",
    targets = {
        "//tplp/...": "",
    },
)

refresh_compile_commands(
    name = "refresh_compile_commands_tests",
    exclude_headers = "external",
    targets = {
        "//tplp/bus:dma_program_test": "",
    },
)
