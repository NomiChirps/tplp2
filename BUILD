load("@rules_pico//pico:defs.bzl", "pico_binary")
load("@rules_pico//pico:defs.bzl", "pico_add_uf2_output")

exports_files(["FreeRTOSConfig.h"])

pico_binary(
    name = "hello.elf",
    srcs = [
        "hello.cpp",
    ],
    deps = [
        "@FreeRTOS-Kernel//:kernel",
        "@FreeRTOS-Kernel//:port_lib",
        "@FreeRTOS-Kernel//:heap",
        "@rules_pico//pico:pico_stdlib",
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
