load("@rules_pico//pico:defs.bzl", "pico_add_uf2_output", "pico_binary")
load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")

exports_files(["FreeRTOSConfig.h"])

pico_binary(
    name = "hello.elf",
    srcs = [
        "hello.cpp",
    ],
    deps = [
        "@FreeRTOS-Kernel//:FreeRTOS",
        "@FreeRTOS-Kernel//:task",
        "@rules_pico//pico:pico_stdlib",
        "@adafruit-sharp-memory-display//:adafruit-sharp-memory-display",

        # Choose a heap implementation
        "@FreeRTOS-Kernel//:heap_1",
    ],
)

pico_add_uf2_output(
    name = "hello.uf2",
    input = "hello.elf",
)

cc_library(
    name = "FreeRTOSConfig",
    hdrs = ["FreeRTOSConfig.h"],
)

# For VSCode
refresh_compile_commands(
    name = "refresh_compile_commands",
    targets = {
        "//:hello.elf": "",
    },
)
