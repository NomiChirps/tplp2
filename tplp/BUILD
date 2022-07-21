load("@rules_pico//pico:defs.bzl", "pico_add_uf2_output", "pico_binary", "pico_add_map_output", "pico_pio_header")

pico_binary(
    name = "firmware",
    srcs = [
        "main.cc",
        "ws2812.pio.h",
    ],
    deps = [
        "@FreeRTOS-Kernel//:FreeRTOS",
        "@FreeRTOS-Kernel//:task",
        "@FreeRTOS-Kernel//:queue",
        "@rules_pico//pico:pico_stdlib",
        "@rules_pico//pico:hardware_gpio",

        # Choose a heap implementation
        "@FreeRTOS-Kernel//:heap_1",
        ":util",
        ":ws2812",
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

pico_add_map_output(
    name = "firmware.map",
    input = "firmware",
)

pico_pio_header(
    name = "ws2812.pio.h",
    input = "ws2812.pio",
)

cc_library(
    name = "ws2812",
    srcs = [
        "ws2812.cc",
        "ws2812.pio.h",
    ],
    hdrs = [
        "ws2812.h",
    ],
    deps = [
        "@rules_pico//pico:hardware_pio",
        "@rules_pico//pico:hardware_clocks",
    ],
)