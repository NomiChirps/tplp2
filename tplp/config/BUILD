cc_library(
    name = "hw",
    hdrs = ["hw.h"],
    # For loose coupling reasons, only the startup task should have direct references to hardware parameters.
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
    # Private visibility, but it's actually injected by a command line flag.
    visibility = ["//visibility:private"],
    deps = ["@rules_pico//pico:pico_time"],
)

cc_library(
    name = "constants",
    hdrs = ["constants.h"],
    visibility = ["//visibility:public"],
    deps = [],
)

cc_library(
    name = "params",
    srcs = ["params.cc"],
    hdrs = ["params.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//absl/strings",
        "//picolog",
        "//picolog:status",
        "//picolog:status_macros",
        "//picolog:statusor",
        "//tplp:numbers",
    ],
)

cc_library(
    name = "public_params",
    hdrs = ["public_params.h"],
    visibility = ["//visibility:public"],
    deps = ["//tplp/config:params"],
)

cc_library(
    name = "params_storage",
    srcs = ["params_storage.cc"],
    hdrs = ["params_storage.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":params",
        "//absl/strings",
        "//picolog",
        "//picolog:status",
        "//picolog:status_macros",
        "//picolog:statusor",
        "//tplp/fs",
    ],
)
