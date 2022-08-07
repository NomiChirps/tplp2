load("@rules_pico//pico:defs.bzl", "pico_add_map_output", "pico_add_uf2_output", "pico_binary", "pico_build_with_config", "pico_pio_header")

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
    visibility = ["//lib:__pkg__"],
)

cc_library(
    name = "lv_tick_custom",
    srcs = ["lv_tick_custom.cc"],
    hdrs = ["lv_tick_custom.h"],
    deps = [
        "@rules_pico//pico:pico_time",
    ],
)

pico_build_with_config(
    name = "firmware.elf",

    # Thanks Adafruit I love you <3
    #board = "adafruit_feather_rp2040",
    # Using a basic Pico for now, until the debug headers come in for the Feather
    board = "pico",
    double_impl = "none",
    float_impl = "none",
    input = ":firmware_unconfigured",
    rtos_adapter_enable = True,
    rtos_adapter_header = "@FreeRTOS//:freertos_sdk_config",
    rtos_adapter_header_name = "FreeRTOS/freertos_sdk_config.h",
    stdio_semihosting = False,
    stdio_uart = False,
    # TODO: we might want to disable stdio_usb later; it uses an interrupt every 1ms
    # FIXME: need to disable this for debug
    stdio_usb = True,
    visibility = ["//visibility:public"],
)

pico_binary(
    name = "firmware_unconfigured",
    srcs = [
        "main.cc",
    ],
    visibility = ["//visibility:private"],
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
        ":FreeRTOSConfig",
    ],
)

cc_library(
    name = "RuntimeStats",
    srcs = ["RuntimeStats.cc"],
    hdrs = ["RuntimeStats.h"],
    deps = [
        ":time",
        ":tplp_config",
        "//picolog",
        "@FreeRTOS",
        "//lib:lvgl",
    ],
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
    ],
)

# TODO: move to a library. or yeet it.
cc_library(
    name = "thread_local",
    hdrs = ["thread_local.h"],
    deps = [
        "@FreeRTOS",
    ],
)

cc_library(
    name = "SpiManager",
    srcs = ["SpiManager.cc"],
    hdrs = ["SpiManager.h"],
    visibility = [":__subpackages__"],
    deps = [
        ":time",
        ":tplp_config",
        ":types",
        "//picolog",
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
