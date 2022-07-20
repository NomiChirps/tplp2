load("@rules_pico//pico:defs.bzl", "pico_add_uf2_output", "pico_binary")

pico_binary(
    name = "firmware",
    srcs = [
        "main.cc",
    ],
    deps = [
        "@FreeRTOS-Kernel//:FreeRTOS",
        "@FreeRTOS-Kernel//:task",
        "@rules_pico//pico:pico_stdlib",
        "@adafruit-sharp-memory-display//:adafruit-sharp-memory-display",

        # Choose a heap implementation
        "@FreeRTOS-Kernel//:heap_1",
        ":util",
    ],
)

# why yes, i *am* embarrassed about making a "util.h" to dump random stuff into
cc_library(
    name = "util",
    hdrs = [
        "util.h",
    ],
    deps = [
        "@FreeRTOS-Kernel//:FreeRTOS",
    ],
)

pico_add_uf2_output(
    name = "firmware.uf2",
    input = "firmware",
)
