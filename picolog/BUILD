cc_library(
    name = "picolog",
    srcs = ["picolog.cc"],
    hdrs = ["picolog.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":backtrace",
        ":status",
        ":vlog_is_on",
        "@FreeRTOS",
        "@rules_pico//pico:pico_stdlib",
    ],
)

cc_library(
    name = "vlog_is_on",
    hdrs = ["vlog_is_on.h"],
)

cc_library(
    name = "backtrace",
    srcs = ["backtrace.cc"],
    hdrs = ["backtrace.h"],
    target_compatible_with = [
        "@platforms//cpu:armv6-m",
    ],
)

cc_test(
    name = "vlog_is_on_test",
    srcs = ["vlog_is_on_test.cc"],
    deps = [":vlog_is_on"],
)

cc_library(
    name = "status",
    hdrs = ["status.h"],
    srcs = ["status.cc"],
    deps = [],
)
