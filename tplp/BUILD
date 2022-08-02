load("@rules_pico//pico:defs.bzl", "pico_add_map_output", "pico_add_uf2_output", "pico_binary", "pico_pio_header")

cc_library(
    name = "FreeRTOSConfig",
    hdrs = ["FreeRTOSConfig.h"],
    strip_include_prefix = "/tplp",
    deps = ["@FreeRTOS//:rp2040_config"],
)

cc_library(
    name = "lv_conf",
    hdrs = ["lv_conf.h"],
    strip_include_prefix = "/tplp",
    deps = [":lv_tick_custom"],
)

cc_library(
    name = "lv_tick_custom",
    srcs = ["lv_tick_custom.cc"],
    hdrs = ["lv_tick_custom.h"],
    deps = [
        "@rules_pico//pico:pico_time",
    ],
)

pico_binary(
    name = "firmware",
    srcs = [
        "main.cc",
    ],
    deps = [
        ":RuntimeStats",
        ":SpiManager",
        ":hook_impl",
        ":time",
        ":tplp_config",
        ":types",
        ":ws2812",
        "//tplp/HX8357",
        "//tplp/SharpLCD",
        "//tplp/graphics:lvgl_init",
        "@FreeRTOS",
        "@rules_pico//pico:pico_stdlib",
    ],
)

cc_library(
    name = "hook_impl",
    srcs = ["hook_impl.cc"],
    deps = [
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
        ":FreeRTOSConfig",
    ],
)

cc_library(
    name = "RuntimeStats",
    srcs = ["RuntimeStats.cc"],
    hdrs = ["RuntimeStats.h"],
    deps = [
        ":logging",
        ":time",
        ":tplp_config",
        "@FreeRTOS",
        "@lvgl",
    ],
)

cc_library(
    name = "logging",
    srcs = ["logging.cc"],
    hdrs = ["logging.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@FreeRTOS",
        "@rules_pico//pico:pico_stdlib",
        ":backtrace",
    ],
)

cc_library(
    name = "backtrace",
    srcs = ["backtrace.cc"],
    hdrs = ["backtrace.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "tplp_config",
    hdrs = ["tplp_config.h"],
    visibility = [":__subpackages__"],
    deps = [
        ":FreeRTOSConfig",
        ":types",
    ],
)

cc_library(
    name = "types",
    hdrs = ["types.h"],
    visibility = [":__subpackages__"],
    deps = [
        "@NamedType",
        "@rules_pico//pico:hardware_base",  # should be pico_types, but it's not visible
    ],
)

cc_library(
    name = "thread_local",
    hdrs = ["thread_local.h"],
    deps = [
        ":tplp_config",
        "@FreeRTOS",
        "@etl//:flat_map",
    ],
)

cc_library(
    name = "SpiManager",
    srcs = ["SpiManager.cc"],
    hdrs = ["SpiManager.h"],
    visibility = [":__subpackages__"],
    deps = [
        ":logging",
        ":thread_local",
        ":time",
        ":types",
        "@FreeRTOS",
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
