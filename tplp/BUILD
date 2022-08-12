load("@rules_pico//pico:defs.bzl", "pico_add_map_output", "pico_add_uf2_output", "pico_binary", "pico_pio_header")

pico_binary(
    name = "firmware.elf",
    srcs = [
        "main.cc",
    ],
    visibility = ["//visibility:private"],
    deps = [
        ":RuntimeStats",
        ":SpiManager",
        ":hook_impl",
        ":time",
        ":types",
        ":ws2812",
        "//tplp/HX8357",
        "//tplp/SharpLCD",
        "//tplp/config:tplp_config",
        "//tplp/graphics:lvgl_init",
        "//tplp/ui:main",
        "@FreeRTOS",
        "@rules_pico//pico:pico_stdlib",
    ],
)

cc_library(
    name = "hook_impl",
    srcs = ["hook_impl.cc"],
    deps = [
        "//picolog",
        "@FreeRTOS",
        "@rules_pico//pico:pico_time",
    ],
    alwayslink = 1,
)

cc_library(
    name = "time",
    hdrs = [
        "time.h",
    ],
    deps = [
        "@FreeRTOS//config:FreeRTOSConfig",
    ],
)

cc_library(
    name = "RuntimeStats",
    srcs = ["RuntimeStats.cc"],
    hdrs = ["RuntimeStats.h"],
    deps = [
        ":time",
        "//picolog",
        "//tplp/config:tplp_config",
        "@FreeRTOS",
        "@lvgl",
    ],
)

cc_library(
    name = "types",
    hdrs = ["types.h"],
    visibility = [":__subpackages__"],
    deps = [
        "@NamedType",
    ],
)

cc_library(
    name = "SpiManager",
    srcs = ["SpiManager.cc"],
    hdrs = ["SpiManager.h"],
    visibility = [":__subpackages__"],
    deps = [
        ":time",
        ":types",
        "//picolog",
        "//tplp/config:tplp_config",
        "@FreeRTOS",
        "@rules_pico//pico:hardware_dma",
        "@rules_pico//pico:hardware_gpio",
        "@rules_pico//pico:hardware_spi",
    ],
)

pico_add_uf2_output(
    name = "firmware.uf2",
    input = "firmware.elf",
    visibility = ["//visibility:public"],
)

# TODO: check this out
pico_add_map_output(
    name = "firmware.map",
    input = "firmware.elf",
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
