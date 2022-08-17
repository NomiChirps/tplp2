cc_library(
    name = "hal",
    hdrs = [
        "backtrace.h",
        "hal.h",
    ],
    visibility = ["//picolog:__subpackages__"],
    deps = select({
        # TODO: doesn't rules_pico have a platform tag we can use?
        "@platforms//cpu:armv6-m": [
            ":hal_pico",
            ":backtrace_arm",
        ],
        "//conditions:default": [
            ":hal_posix",
            ":backtrace_none",
        ],
    }),
)

cc_library(
    name = "hal_pico",
    srcs = ["hal_pico.cc"],
    hdrs = ["hal.h"],
    target_compatible_with = [
        "@platforms//cpu:armv6-m",
    ],
    deps = ["@rules_pico//pico:pico_time"],
)

cc_library(
    name = "backtrace_arm",
    srcs = ["backtrace_arm.cc"],
    hdrs = ["backtrace.h"],
    target_compatible_with = [
        "@platforms//cpu:armv6-m",
    ],
)

cc_library(
    name = "hal_posix",
    srcs = ["hal_posix.cc"],
    hdrs = ["hal.h"],
    target_compatible_with = select({
        "@platforms//os:none": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "backtrace_none",
    srcs = ["backtrace_none.cc"],
    hdrs = ["backtrace.h"],
)
