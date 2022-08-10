cc_library(
    name = "tplp_config",
    hdrs = ["tplp_config.h"],
    visibility = ["//tplp:__subpackages__"],
    deps = [
        ":FreeRTOSConfig",
        "//tplp:types",
    ],
)

cc_library(
    name = "FreeRTOSConfig",
    hdrs = ["FreeRTOSConfig.h"],
    strip_include_prefix = "/tplp/config",
    deps = ["@FreeRTOS//ports/GCC_RP2040:rp2040_config"],
)

cc_library(
    name = "lv_conf",
    hdrs = ["lv_conf.h"],
    strip_include_prefix = "/tplp/config",
    visibility = ["//lib:__pkg__"],
    deps = ["@rules_pico//pico:pico_time"],
)
