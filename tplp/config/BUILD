cc_library(
    name = "pins",
    hdrs = ["pins.h"],
    # For loose coupling reasons, only the startup task should have direct references to all the pin assignments.
    visibility = ["//tplp/main:__pkg__"],
    deps = [
        "//tplp/bus:types",
    ],
)

cc_library(
    name = "tasks",
    hdrs = ["tasks.h"],
    # For loose coupling reasons, only the startup task should have direct references to all the tasks' config info.
    visibility = ["//tplp/main:__pkg__"],
    deps = [
        ":FreeRTOSConfig",
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

cc_library(
    name = "params",
    hdrs = ["params.h"],
    visibility = ["//visibility:public"],
    deps = [],
)
