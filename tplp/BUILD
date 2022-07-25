load("@rules_pico//pico:defs.bzl", "pico_add_map_output", "pico_add_uf2_output", "pico_binary", "pico_pio_header")

cc_library(
    name = "FreeRTOSConfig",
    hdrs = ["FreeRTOSConfig.h"],
    strip_include_prefix = "/tplp",
)

pico_binary(
    name = "firmware",
    srcs = [
        "main.cc",
        "ws2812.pio.h",
    ],
    deps = [
        ":SpiManager",
        ":config",
        ":types",
        ":util",
        ":ws2812",
        "//tplp/SharpLCD",
        "@FreeRTOS-Kernel//:FreeRTOS",
        "@rules_pico//pico:hardware_gpio",
        "@rules_pico//pico:hardware_spi",
        "@rules_pico//pico:pico_stdlib",
    ],
)

# TODO: "util.h" is an embarrassment
cc_library(
    name = "util",
    hdrs = [
        "util.h",
    ],
    deps = [
        ":FreeRTOSConfig",
    ],
)

cc_library(
    name = "assert",
    hdrs = ["assert.h"],
    visibility = [":__subpackages__"],
    deps = [
        "@rules_pico//pico:pico_stdlib",
    ],
)

cc_library(
    name = "logging",
    hdrs = ["logging.h"],
    deps = [
        "@fmtlib//:format",
    ],
)

cc_library(
    name = "config",
    hdrs = ["config.h"],
    deps = [
        ":types",
    ],
)

cc_library(
    name = "types",
    hdrs = ["types.h"],
    deps = [
        "@rules_pico//pico:hardware_base",  # should be pico_types, but it's not visible
    ],
)

cc_library(
    name = "thread_local",
    hdrs = ["thread_local.h"],
    deps = [
        ":config",
        "@FreeRTOS-Kernel//:FreeRTOS",
        "@etl//:flat_map",
    ],
)

cc_library(
    name = "SpiManager",
    srcs = ["SpiManager.cc"],
    hdrs = ["SpiManager.h"],
    visibility = [":__subpackages__"],
    deps = [
        ":assert",
        ":logging",
        ":thread_local",
        ":types",
        ":util",
        "@FreeRTOS-Kernel//:FreeRTOS",
        "@rules_pico//pico:hardware_dma",
        "@rules_pico//pico:hardware_gpio",
        "@rules_pico//pico:hardware_spi",
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
        "@rules_pico//pico:hardware_clocks",
        "@rules_pico//pico:hardware_pio",
    ],
)
